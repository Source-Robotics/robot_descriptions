"""Compare local MSG physics, clearance queries, and offscreen rendering timings.

Pass the original package, the J1-J3-only package, and the current package.
Rendering includes scene construction and RGB readback at 1280x720; this is
not an end-to-end Haply latency or GUI FPS measurement.
"""

from pathlib import Path
import argparse
import json
import statistics
from time import perf_counter

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
from model_reference import RCB_FROM_MJ_OFFSET,ARM_JOINTS


class BenchmarkModel:
    """Direct MuJoCo benchmark adapter; no motion application dependency."""
    def __init__(self,package,variant='msg',msg_jaws='stock'):
        self.msg_jaws={'solid':'stock','soft-tpu':'tpu-glue'}.get(msg_jaws,msg_jaws)
        self.ssg48_jaws='pinch' if variant=='ssg48' else None
        profile='msg_tpu_glue' if variant=='msg' and self.msg_jaws=='tpu-glue' else variant
        spec=mujoco.MjSpec.from_file(str(package/f'scene_{profile}.xml'))
        spec.add_exclude(bodyname1='base_link',bodyname2='shoulder_link')
        self.model=spec.compile();self.data=mujoco.MjData(self.model)
        self.dof_adr=np.array([self.model.joint(name).dofadr[0] for name in ARM_JOINTS])
        self.qpos_adr=np.array([self.model.joint(name).qposadr[0] for name in ARM_JOINTS])
        self._fromto=np.zeros(6)
        m=self.model
        active=np.flatnonzero((m.geom_contype!=0)|(m.geom_conaffinity!=0))
        robot=[g for g in active if m.geom_bodyid[g]!=0];floor=[g for g in active if m.geom_bodyid[g]==0]
        def welded(body):
            while body and m.body_jntnum[body]==0:body=m.body_parentid[body]
            return body
        candidates=[]
        for i,a in enumerate(robot):
            for b in robot[i+1:]:
                x,y=welded(m.geom_bodyid[a]),welded(m.geom_bodyid[b])
                if x==y or m.body_parentid[x]==y or m.body_parentid[y]==x:continue
                names={m.body(m.geom_bodyid[a]).name,m.body(m.geom_bodyid[b]).name}
                if names=={'base_link','shoulder_link'}:continue
                candidates.append((a,b))
            if welded(m.geom_bodyid[a]):candidates.extend((a,b) for b in floor)
        mujoco.mj_forward(m,self.data)
        self._gap_pairs=[(a,b) for a,b in candidates if mujoco.mj_geomDistance(m,self.data,a,b,.5,self._fromto)>=1e-4]

    def set_joints(self,q):
        self.data.qpos[self.qpos_adr]=q-RCB_FROM_MJ_OFFSET
        self.data.qvel[:]=0;mujoco.mj_forward(self.model,self.data)

    def gap_info_all(self,influence=.03):
        m,d=self.model,self.data;result=[]
        for a,b in self._gap_pairs:
            gap=mujoco.mj_geomDistance(m,d,a,b,influence,self._fromto)
            if gap>=influence:continue
            p,q=self._fromto[:3],self._fromto[3:];direction=q-p
            length=np.linalg.norm(direction);gradient=None
            if gap>0 and length>=1e-9:
                ja=np.zeros((3,m.nv));jb=np.zeros((3,m.nv))
                mujoco.mj_jac(m,d,ja,None,p,m.geom_bodyid[a]);mujoco.mj_jac(m,d,jb,None,q,m.geom_bodyid[b])
                gradient=(direction/length)@(jb[:,self.dof_adr]-ja[:,self.dof_adr])
            result.append((float(gap),(m.body(m.geom_bodyid[a]).name,m.body(m.geom_bodyid[b]).name),gradient))
        return sorted(result,key=lambda item:item[0])


def timed_ms(function, count):
    begin = perf_counter()
    for _ in range(count):
        function()
    return (perf_counter() - begin) * 1000 / count


def benchmark(packages, camera_azimuth=135, rounds=7, frames_per_round=20, msg_jaws=None, variant='msg'):
    twins, renderers, samples = {}, {}, {}
    try:
        for name, package in packages.items():
            twin = BenchmarkModel(package.resolve(),variant,msg_jaws=(msg_jaws or {}).get(name,'stock'))
            twins[name] = twin
            m, d = twin.model, twin.data
            m.vis.global_.offwidth, m.vis.global_.offheight = 1280, 720
            mujoco.mj_resetDataKeyframe(m, d, 1)
            for _ in range(500):
                mujoco.mj_step(m, d)
            renderer = mujoco.Renderer(m, width=1280, height=720)
            camera = mujoco.MjvCamera()
            camera.lookat[:] = [0, 0, .18]
            camera.distance, camera.azimuth, camera.elevation = .9, camera_azimuth, -20
            renderers[name] = (renderer, camera)
            for _ in range(6):
                renderer.update_scene(d, camera)
                renderer.render()
            samples[name] = {'step_ms': [], 'gap_info_all_ms': [], 'frame_ms': []}

        rng = np.random.default_rng(2710)
        for round_id in range(rounds):
            for name in rng.permutation(list(packages)):
                twin = twins[name]
                m, d = twin.model, twin.data
                renderer, camera = renderers[name]
                samples[name]['step_ms'].append(timed_ms(lambda: mujoco.mj_step(m, d), 4000))
                twin.set_joints(np.radians([90, -105, 165, 0, -30, 180]))
                samples[name]['gap_info_all_ms'].append(timed_ms(twin.gap_info_all, 200))
                def frame():
                    renderer.update_scene(d, camera)
                    renderer.render()
                samples[name]['frame_ms'].append(timed_ms(frame, frames_per_round))
            print(f'Benchmark round {round_id + 1}/{rounds} complete', flush=True)

        report = {'mujoco_version': mujoco.__version__, 'resolution': [1280, 720],
                  'camera_azimuth': camera_azimuth, 'rounds': rounds, 'frames_per_round': frames_per_round,
                  'measurement': 'Standalone MuJoCo adapter, interleaved batches. Frame includes scene update/RGB readback, not GUI or Haply I/O. Clearance uses the same pair policy, but this is not a timing of the motion application.',
                  'models': {}}
        for name, twin in twins.items():
            model = twin.model
            active = (model.geom_contype != 0) | (model.geom_conaffinity != 0)
            report['models'][name] = {
                'variant':variant, 'msg_jaws': twin.msg_jaws, 'ssg48_jaws':twin.ssg48_jaws,
                'total_geoms': model.ngeom, 'collision_geoms': int(active.sum()),
                'clearance_pairs': len(twin._gap_pairs),
                'mesh_triangles_in_memory': int(model.mesh_facenum.sum()),
                'timings': {metric: {'median_ms': statistics.median(values),
                                     'min_ms': min(values), 'max_ms': max(values), 'samples_ms': values}
                            for metric, values in samples[name].items()},
            }
        return report
    finally:
        for renderer, _ in renderers.values():
            renderer.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--original', type=Path,help='Optional historical comparison package')
    parser.add_argument('--motors-only', type=Path,help='Optional historical comparison package')
    parser.add_argument('--current', type=Path, default=ROOT / 'mujoco/par6')
    parser.add_argument('--output', type=Path,default=ROOT/'reports/appearance/benchmark.json')
    parser.add_argument('--rounds',type=int,default=7)
    parser.add_argument('--frames-per-round',type=int,default=20)
    args = parser.parse_args()
    packages={'current':args.current}
    if args.original:packages['original']=args.original
    if args.motors_only:packages['motors_only']=args.motors_only
    report = benchmark(packages,rounds=args.rounds,frames_per_round=args.frames_per_round)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    for name, result in report['models'].items():
        print(name, {metric: round(timing['median_ms'], 5) for metric, timing in result['timings'].items()})
