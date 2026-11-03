# SPDX-License-Identifier: Apache-2.0
"""Exercise a relocated Linux MoveIt install: real planning and mock execution."""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

QT_APPLICATION = None
ARM = ['base_joint','shoulder_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
TOOLS = [('flange','default','flange'), ('msg','solid','msg'), ('msg','soft_tpu','msg_tpu_glue'),
         ('ssg48','solid','ssg48'), ('ssg48','fin_ray','ssg48_stock'), ('pneumatic','default','pneumatic')]


class Probe:
    def __init__(self, data):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from moveit_msgs.srv import GetMotionPlan, GetStateValidity, GetPositionFK, GetPositionIK, ApplyPlanningScene
        from moveit_msgs.action import ExecuteTrajectory
        from rclpy.action import ActionClient
        self.rclpy = rclpy; self.data = data; self.current = {}
        self.context = Context(); rclpy.init(context=self.context)
        self.node = rclpy.create_node('moveit_integration_probe', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context); self.executor.add_node(self.node)
        self.subscription = self.node.create_subscription(JointState, '/joint_states',
            lambda msg: self.current.update(zip(msg.name,msg.position)), qos_profile_sensor_data)
        self.services = {}
        for name, kind in [('/plan_kinematic_path',GetMotionPlan), ('/check_state_validity',GetStateValidity),
            ('/compute_fk',GetPositionFK), ('/compute_ik',GetPositionIK), ('/apply_planning_scene',ApplyPlanningScene)]:
            client = self.node.create_client(kind,name)
            assert client.wait_for_service(timeout_sec=60), f'Missing service: {name}'
            self.services[name] = client
        self.action = ActionClient(self.node, ExecuteTrajectory, '/execute_trajectory')
        assert self.action.wait_for_server(timeout_sec=30), 'Execution action missing'
        self.until(lambda: set(data['position_limits']) <= self.current.keys(), timeout=30)

    def until(self, predicate, timeout=30):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=.02)
            if predicate():return
        raise AssertionError('Timed out waiting for ROS result/state')

    def future(self, future, timeout=30):
        self.until(future.done, timeout)
        return future.result()

    def call(self, name, request):
        return self.future(self.services[name].call_async(request))

    def state(self, values):
        from moveit_msgs.msg import RobotState
        state = RobotState(); positions = dict(self.data['initial_positions']); positions.update(values)
        if self.data['has_gripper']:
            # Infer the selected URDF mimic sign from its endpoint ranges.
            sign = 1. if self.data['position_limits']['left_finger_joint'][0] == 0 else -1.
            positions['right_finger_joint'] = sign*positions['left_finger_joint']
        state.joint_state.name = list(positions); state.joint_state.position = list(positions.values())
        state.joint_state.header.stamp = self.node.get_clock().now().to_msg()
        return state

    def validity(self, values):
        from moveit_msgs.srv import GetStateValidity
        request = GetStateValidity.Request(); request.robot_state = self.state(values)
        return self.call('/check_state_validity',request)

    def fk(self, values):
        from moveit_msgs.srv import GetPositionFK
        request = GetPositionFK.Request(); request.header.frame_id='world';request.fk_link_names=['tcp']
        request.robot_state = self.state(values)
        result = self.call('/compute_fk',request)
        assert result.error_code.val == 1, result
        return result.pose_stamped[0]

    def plan(self, start, goal, group='arm', pose=None):
        from moveit_msgs.srv import GetMotionPlan
        from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint
        from shape_msgs.msg import SolidPrimitive
        from geometry_msgs.msg import Pose
        request = GetMotionPlan.Request(); motion = request.motion_plan_request
        motion.group_name=group; motion.pipeline_id='ompl';motion.planner_id='RRTConnect'
        motion.allowed_planning_time=5.; motion.num_planning_attempts=1
        motion.max_velocity_scaling_factor=.5;motion.max_acceleration_scaling_factor=.5
        motion.start_state=self.state(start)
        constraint = Constraints()
        if pose is None:
            for name,value in goal.items():
                joint = JointConstraint();joint.joint_name=name;joint.position=float(value)
                joint.tolerance_above=1e-5;joint.tolerance_below=1e-5;joint.weight=1.
                constraint.joint_constraints.append(joint)
        else:
            position = PositionConstraint();position.header=pose.header;position.link_name='tcp';position.weight=1.
            primitive=SolidPrimitive();primitive.type=SolidPrimitive.SPHERE;primitive.dimensions=[.001]
            region=Pose();region.position=pose.pose.position;region.orientation.w=1.
            position.constraint_region.primitives=[primitive];position.constraint_region.primitive_poses=[region]
            orientation=OrientationConstraint();orientation.header=pose.header;orientation.link_name='tcp'
            orientation.orientation=pose.pose.orientation;orientation.weight=1.
            orientation.absolute_x_axis_tolerance=.01;orientation.absolute_y_axis_tolerance=.01;orientation.absolute_z_axis_tolerance=.01
            constraint.position_constraints=[position];constraint.orientation_constraints=[orientation]
        motion.goal_constraints=[constraint]
        return self.call('/plan_kinematic_path',request).motion_plan_response

    def check_trajectory(self, response, start):
        assert response.error_code.val == 1, response.error_code
        trajectory = response.trajectory.joint_trajectory
        assert len(trajectory.points) >= 2
        assert set(trajectory.joint_names) <= self.data['position_limits'].keys()
        last_time=-1.; previous=None; count=0
        for point in trajectory.points:
            stamp=point.time_from_start.sec+point.time_from_start.nanosec*1e-9
            assert stamp > last_time; last_time=stamp
            values=dict(zip(trajectory.joint_names,point.positions))
            for index,(name,q) in enumerate(values.items()):
                low,high=self.data['position_limits'][name]
                assert low-1e-5 <= q <= high+1e-5, (name,q,low,high)
                vmax,amax=(.5,1.) if name in ARM else (.03,.1)
                if point.velocities:assert abs(point.velocities[index]) <= vmax+1e-4
                if point.accelerations:assert abs(point.accelerations[index]) <= amax+1e-3
            # Recheck every waypoint and denser interpolated states through the
            # independent state-validity service, with the other joints fixed.
            n=1 if previous is None else max(1,math.ceil(max(abs(values[k]-previous[k])/(.04 if k in ARM else .001) for k in values)))
            for step in range(1,n+1):
                sample=dict(start);sample.update({k:v if previous is None else previous[k]+(v-previous[k])*step/n for k,v in values.items()})
                result=self.validity(sample)
                assert result.valid, [(c.contact_body_1,c.contact_body_2) for c in result.contacts]
                count+=1
            previous=values
        return count

    def execute(self, response):
        from moveit_msgs.action import ExecuteTrajectory
        goal=ExecuteTrajectory.Goal();goal.trajectory=response.trajectory
        handle=self.future(self.action.send_goal_async(goal));assert handle.accepted
        result=self.future(handle.get_result_async(),timeout=40)
        assert result.status == 4 and result.result.error_code.val == 1, result
        final=dict(zip(response.trajectory.joint_trajectory.joint_names,response.trajectory.joint_trajectory.points[-1].positions))
        self.until(lambda: all(abs(self.current.get(k,1e9)-v)<(1e-3 if k in ARM else 1e-5) for k,v in final.items()))
        if self.data['has_gripper']:
            sign=1. if self.data['position_limits']['left_finger_joint'][0] == 0 else -1.
            self.until(lambda: abs(self.current['right_finger_joint']-sign*self.current['left_finger_joint']) < 1e-8)

    def close(self):
        self.action.destroy();self.executor.shutdown();self.node.destroy_node();self.rclpy.shutdown(context=self.context)


def run_case(index,family,tool,jaws,profile,output,gui=False):
    os.environ['ROS_DOMAIN_ID']=str(90+index)
    from ament_index_python.packages import get_package_share_directory
    import yaml
    share=Path(get_package_share_directory(family+'_moveit_config'))
    data=json.loads((share/f'config/profiles/{profile}.json').read_text())
    name=f'{index:02d}_{family}_{profile}'+('_gui' if gui else '')
    log_path=output/(name+'.log');probe=None
    with log_path.open('w') as log:
        process=subprocess.Popen(['ros2','launch',family+'_moveit_config','demo.launch.py',
            'end_effector:='+tool,'jaws:='+jaws,'rviz:='+str(gui).lower()],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            probe=Probe(data)
            start=dict(data['initial_positions']);assert all(abs(probe.current[n]-q)<1e-7 for n,q in start.items())
            for state in ['home','ready']:
                result=probe.validity(data['named_states'][state])
                assert result.valid, (state,[(c.contact_body_1,c.contact_body_2) for c in result.contacts])
            ready=data['named_states']['ready']
            plan=probe.plan(start,ready);checks=probe.check_trajectory(plan,start);probe.execute(plan)
            # Exercise actual IK for the selected TCP, then plan a pose target.
            target=dict(ready);target['base_joint']+=.12;target['shoulder_joint']+=.03
            pose=probe.fk(target)
            from moveit_msgs.srv import GetPositionIK
            request=GetPositionIK.Request();request.ik_request.group_name='arm'
            request.ik_request.ik_link_name='tcp';request.ik_request.pose_stamped=pose
            request.ik_request.robot_state=probe.state(ready);request.ik_request.avoid_collisions=True
            request.ik_request.timeout.sec=1
            solved=probe.call('/compute_ik',request)
            assert solved.error_code.val == 1, solved.error_code
            solution=dict(zip(solved.solution.joint_state.name,solved.solution.joint_state.position))
            solved_pose=probe.fk(solution)
            a,b=pose.pose,solved_pose.pose
            assert sum((getattr(a.position,k)-getattr(b.position,k))**2 for k in ['x','y','z'])**.5 < 1e-4
            qa=[getattr(a.orientation,k) for k in ['x','y','z','w']];qb=[getattr(b.orientation,k) for k in ['x','y','z','w']]
            assert 1-abs(sum(x*y for x,y in zip(qa,qb))) < 1e-5
            pose_plan=probe.plan(dict(probe.current),{},pose=pose)
            checks+=probe.check_trajectory(pose_plan,dict(probe.current));probe.execute(pose_plan)
            gripper_states=[]
            if data['has_gripper']:
                for state in ['closed','open']:
                    before=dict(probe.current);goal=data['named_states'][state]
                    result=probe.validity(dict(before,**goal))
                    assert result.valid, (state,[(c.contact_body_1,c.contact_body_2) for c in result.contacts])
                    plan=probe.plan(before,goal,'gripper');checks+=probe.check_trajectory(plan,before);probe.execute(plan)
                    gripper_states.append(state)
            # MoveIt versions may reject an out-of-range joint constraint or
            # clamp it to the physical range. Either response must stay bounded.
            invalid=probe.plan(dict(probe.current),dict(ready,base_joint=data['position_limits']['base_joint'][1]+.5))
            out_of_range_response='rejected'
            if invalid.error_code.val == 1:
                checks+=probe.check_trajectory(invalid,dict(probe.current))
                out_of_range_response='clamped_within_limits'
            # Prove that planning-scene obstacles are checked and can be removed.
            from moveit_msgs.srv import ApplyPlanningScene
            from moveit_msgs.msg import CollisionObject
            from shape_msgs.msg import SolidPrimitive
            from geometry_msgs.msg import Pose
            request=ApplyPlanningScene.Request();request.scene.is_diff=True
            box=CollisionObject();box.id='test_blocker';box.header.frame_id='world';box.operation=CollisionObject.ADD
            primitive=SolidPrimitive();primitive.type=SolidPrimitive.BOX;primitive.dimensions=[2.,2.,2.]
            pose_box=Pose();pose_box.orientation.w=1.;box.primitives=[primitive];box.primitive_poses=[pose_box]
            request.scene.world.collision_objects=[box];assert probe.call('/apply_planning_scene',request).success
            assert not probe.validity(probe.current).valid
            blocked=probe.plan(dict(probe.current),ready);assert blocked.error_code.val != 1
            box.operation=CollisionObject.REMOVE;box.primitives=[];box.primitive_poses=[]
            assert probe.call('/apply_planning_scene',request).success
            assert probe.validity(probe.current).valid
            if gui:
                probe.until(lambda: 'moveit_rviz' in {n for n,_ in probe.node.get_node_names_and_namespaces()},timeout=30)
                until=time.monotonic()+5
                while time.monotonic()<until:probe.executor.spin_once(timeout_sec=.05)
                from PyQt5.QtWidgets import QApplication
                global QT_APPLICATION
                QT_APPLICATION=QApplication.instance() or QApplication([]);QT_APPLICATION.processEvents()
                assert QT_APPLICATION.primaryScreen().grabWindow(0).save(str(output/(name+'.png')))
                text=log_path.read_text().lower()
                assert not any(error in text for error in ['error retrieving file','could not load resource',
                    'unable to load mesh','failed to load mesh','failed to load library','could not find class']),text
            result=dict(case=name,family=family,profile=profile,tcp='tcp',joint_plan_executed=True,
                tcp_ik_and_pose_plan_executed=True,trajectory_states_rechecked=checks,gripper_presets_executed=gripper_states,
                mimic_feedback_checked=data['has_gripper'],out_of_range_goal_response=out_of_range_response,obstacle_blocked_plan_rejected=True,
                gui=gui,passed=True)
            print(json.dumps(result),flush=True)
            return result
        except Exception:
            print(log_path.read_text()[-16000:],flush=True)
            raise
        finally:
            if probe:probe.close()
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=5)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    results=[]
    for family in ['par6','parol6']:
        for tool,jaws,profile in TOOLS:
            results.append(run_case(len(results),family,tool,jaws,profile,args.output))
    for family,tool,jaws,profile in [('par6','msg','soft_tpu','msg_tpu_glue'),('parol6','ssg48','fin_ray','ssg48_stock')]:
        results.append(run_case(len(results),family,tool,jaws,profile,args.output,gui=True))
    result=dict(ros_distro=os.environ['ROS_DISTRO'],cases=results,all_12_variants_checked=True,gui_cases=2)
    (args.output/'validation.json').write_text(json.dumps(result,indent=2)+'\n')
    print('All 14 installed MoveIt planning, IK, collision and mock execution cases passed.',flush=True)


if __name__=='__main__':main()
