# SPDX-License-Identifier: Apache-2.0
"""Linux integration checks against a relocated colcon install (requires ROS 2)."""
import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

_QT_APPLICATION = None


def product_cases():
    tools = [('flange', 'default'), ('msg', 'solid'), ('msg', 'soft_tpu'),
             ('ssg48', 'solid'), ('ssg48', 'fin_ray'), ('pneumatic', 'default')]
    return [(family, tool, jaw) for family in ['par6', 'parol6'] for tool, jaw in tools]+[
        ('msg', 'msg', 'solid'), ('msg', 'msg', 'soft_tpu'),
        ('ssg48', 'msg', 'solid'), ('ssg48', 'msg', 'fin_ray')]


def multiply(a, b):
    x,y,z,w = a; X,Y,Z,W = b
    return [w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X, w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z]


def expected_transform(joint, values):
    origin = joint.find('origin')
    xyz = [float(x) for x in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()]
    r,p,y = [float(x)/2 for x in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()]
    sr,cr,sp,cp,sy,cy = math.sin(r),math.cos(r),math.sin(p),math.cos(p),math.sin(y),math.cos(y)
    q = [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]
    position = values.get(joint.get('name'), 0.)
    axis = joint.find('axis')
    axis = [float(x) for x in (axis.get('xyz') if axis is not None else '1 0 0').split()]
    if joint.get('type') == 'prismatic':
        offset = multiply(multiply(q, [v*position for v in axis]+[0]), [-q[0],-q[1],-q[2],q[3]])
        xyz = [a+b for a,b in zip(xyz, offset[:3])]
    elif joint.get('type') in ('revolute', 'continuous'):
        q = multiply(q, [v*math.sin(position/2) for v in axis]+[math.cos(position/2)])
    return xyz, q


def transform_matches(actual, joint, values):
    xyz, quat = expected_transform(joint, values)
    a = actual.transform.translation; b = actual.transform.rotation
    actual_xyz, actual_q = [a.x,a.y,a.z], [b.x,b.y,b.z,b.w]
    return (actual.header.frame_id == joint.find('parent').get('link') and
            max(abs(a-b) for a,b in zip(actual_xyz,xyz)) < 1e-9 and
            min(sum(abs(a-sign*b) for a,b in zip(actual_q,quat)) for sign in [-1,1]) < 1e-9)


def run_case(index, family, tool, jaws, output, gui=False, external=False):
    os.environ['ROS_DOMAIN_ID'] = str(50+index)
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
    from ament_index_python.packages import get_package_share_directory
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    name = f'{index:02d}_{family}_{tool}_{jaws}'+('_gui' if gui else '_external' if external else '')
    share = Path(get_package_share_directory(family+'_description'))
    spec = importlib.util.spec_from_file_location('installed_description_helper', share/'scripts/robot_description.py')
    helper = importlib.util.module_from_spec(spec); spec.loader.exec_module(helper)
    entry = helper.select_model(family, tool, jaws); root = ET.parse(share/entry).getroot()
    expected_joints = {j.get('name'): j for j in root.findall('joint') if j.get('type') != 'fixed'}
    desired = {}
    for joint in expected_joints.values():
        if joint.find('mimic') is not None:continue
        desired[joint.get('name')] = 0.
        if external:
            limit = joint.find('limit')
            desired[joint.get('name')] = ((float(limit.get('lower'))+float(limit.get('upper')))/2
                                         if joint.get('type') == 'prismatic' else .2)
    values = dict(desired)
    for joint in expected_joints.values():
        mimic = joint.find('mimic')
        if mimic is not None:
            values[joint.get('name')] = values[mimic.get('joint')]*float(mimic.get('multiplier','1'))+float(mimic.get('offset','0'))
    context = Context(); rclpy.init(context=context)
    node = rclpy.create_node('description_integration_probe', context=context)
    executor = SingleThreadedExecutor(context=context); executor.add_node(node)
    state = {'description': None, 'joints': None, 'tf': {}}
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    subscriptions = [
        node.create_subscription(String, '/robot_description', lambda msg: state.update(description=msg.data), durable),
        node.create_subscription(JointState, '/joint_states', lambda msg: state.update(joints=dict(zip(msg.name,msg.position))), qos_profile_sensor_data)]
    def receive_tf(msg):
        state['tf'].update({t.child_frame_id:t for t in msg.transforms})
    subscriptions += [node.create_subscription(TFMessage,'/tf',receive_tf,qos_profile_sensor_data),
                      node.create_subscription(TFMessage,'/tf_static',receive_tf,durable)]
    publisher = node.create_publisher(JointState, '/joint_states', 10) if external else None
    command = ['ros2','launch',family+'_description','display.launch.py',f'jaws:={jaws}',
               'gui:='+str(gui).lower(), 'rviz:='+str(gui).lower(),
               'publish_joint_states:='+str(not external).lower()]
    if family in ('par6','parol6'):command.append('end_effector:='+tool)
    log_path = output/(name+'.log')
    with log_path.open('w') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic()+40
            expected_tf = {j.find('child').get('link'):j for j in root.findall('joint')}
            while time.monotonic() < deadline:
                assert process.poll() is None, log_path.read_text()
                if publisher:
                    msg = JointState(); msg.header.stamp = node.get_clock().now().to_msg()
                    msg.name = list(desired); msg.position = list(desired.values()); publisher.publish(msg)
                executor.spin_once(timeout_sec=.05)
                if not state['description'] or not state['joints']:continue
                if not external:
                    # The GUI may choose a centered pose for asymmetric limits.
                    # Check its actual published pose, limits and mimic relation.
                    if set(state['joints']) != set(expected_joints):continue
                    values = dict(state['joints'])
                    for joint in expected_joints.values():
                        q = values[joint.get('name')]; limit = joint.find('limit')
                        if joint.get('type') != 'continuous':
                            assert float(limit.get('lower'))-1e-10 <= q <= float(limit.get('upper'))+1e-10
                        mimic = joint.find('mimic')
                        if mimic is not None:
                            expected = values[mimic.get('joint')]*float(mimic.get('multiplier','1'))+float(mimic.get('offset','0'))
                            assert abs(q-expected)<1e-10
                if not all(child in state['tf'] and transform_matches(state['tf'][child],joint,values)
                           for child,joint in expected_tf.items()):continue
                names = {n for n,_ in node.get_node_names_and_namespaces()}
                if gui and not {'rviz2','joint_state_publisher_gui'} <= names:continue
                if gui and not any(info.node_name == 'rviz2' for info in
                                   node.get_subscriptions_info_by_topic('/robot_description')):continue
                break
            else:
                missing = set(expected_tf)-state['tf'].keys()
                bad_tf = [child for child,joint in expected_tf.items() if child in state['tf'] and
                          not transform_matches(state['tf'][child],joint,values)]
                raise AssertionError(f'{name}: missing={missing}, incorrect={bad_tf}, joints={state["joints"]}\n'+log_path.read_text())
            resolved = ET.fromstring(state['description'])
            assert resolved.get('name') == root.get('name')
            for mesh in resolved.findall('.//mesh'):
                uri = mesh.get('filename'); prefix = f'package://{family}_description/'
                assert uri.startswith(prefix) and (share/uri.removeprefix(prefix)).is_file(),uri
            if gui:
                # Let RViz finish uploading its mesh/texture resources before capturing.
                until = time.monotonic()+4
                while time.monotonic()<until:executor.spin_once(timeout_sec=.05)
                from PyQt5.QtWidgets import QApplication
                global _QT_APPLICATION
                _QT_APPLICATION = QApplication.instance() or QApplication([])
                app = _QT_APPLICATION
                app.processEvents()
                assert app.primaryScreen().grabWindow(0).save(str(output/(name+'.png')))
                text = log_path.read_text().lower()
                assert not any(error in text for error in ['error retrieving file','could not load resource',
                    'could not load model','unable to load mesh','failed to load mesh']),text
            result = {'case':name,'entry':entry,'joint_states_checked':len(expected_joints),
                      'joint_transforms_checked':len(expected_tf),'gui':gui,'external_joint_states':external,'passed':True}
            print(json.dumps(result),flush=True)
            return result
        finally:
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL); process.wait(timeout=5)
            executor.shutdown(); node.destroy_node(); rclpy.shutdown(context=context)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    results=[]
    for case in product_cases():results.append(run_case(len(results),*case,args.output))
    for case in [('par6','pneumatic','default'),('parol6','msg','soft_tpu'),
                 ('msg','msg','solid'),('ssg48','msg','fin_ray')]:
        results.append(run_case(len(results),*case,args.output,external=True))
    for case in [('par6','msg','soft_tpu'),('parol6','ssg48','fin_ray'),
                 ('msg','msg','soft_tpu'),('ssg48','msg','solid')]:
        results.append(run_case(len(results),*case,args.output,gui=True))
    command=['ros2','launch','par6_description','display.launch.py','end_effector:=msg','jaws:=fin_ray','rviz:=false','gui:=false']
    bad=subprocess.run(command,capture_output=True,text=True,timeout=20)
    assert bad.returncode != 0 and 'jaws must be one of' in bad.stdout+bad.stderr
    result={'ros_distro':os.environ['ROS_DISTRO'],'cases':results,'invalid_selection_rejected':True,
            'all_16_variants_checked':True,'gui_cases':4,'external_joint_state_cases':4}
    (args.output/'validation.json').write_text(json.dumps(result,indent=2)+'\n')
    print('All 24 installed ROS 2 launch/TF/mimic/GUI cases passed.',flush=True)


if __name__=='__main__':main()
