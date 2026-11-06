# SPDX-License-Identifier: Apache-2.0
"""Import RAFT's model reference and CAD meshes from a RAFT-teleoperation-arm checkout into source/raft/.

RAFT's firmware generator (firmware/tools/gen_model.py) computes the arm's frames, mass properties, ranges and
keyframes from its DH table and CAD; the same run writes the firmware's model. It exports them, with the robot
description names, to firmware/tools/sim/model/raft_reference.json next to the CAD meshes. This optional operation
copies both here and splits every link mesh into its coloured visual pieces (printed parts, servos, horns, screws).
The normal build does not need the RAFT checkout: it builds from the committed copy.

    python tools/import_raft_reference.py --raft ../RAFT-teleoperation-arm
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/'source/raft'
# CAD mesh name in the RAFT checkout -> robot description link name, and the servos each link holds
CAD = {'base_link': 'base_link', 'link1': 'shoulder_link', 'link2': 'upper_arm_link', 'link3': 'forearm_link',
       'link4': 'wrist_1_link', 'link5': 'wrist_2_link', 'link6': 'wrist_3_link', 'gripper_trigger': 'trigger_link'}
SERVOS = {'base_link': 1, 'shoulder_link': 1, 'upper_arm_link': 0, 'forearm_link': 2, 'wrist_1_link': 1,
          'wrist_2_link': 1, 'wrist_3_link': 1, 'trigger_link': 0}
PART_OF = {'printed': 'structure', 'horn': 'horns', 'servo_screw': 'fasteners', 'm3': 'fasteners'}


def kind(shell):
    """sort one closed shell of a RAFT CAD mesh by its size (the same rules as RAFT's assembly pictures)"""
    e = np.sort(shell.extents*1000)[::-1]
    if shell.is_watertight and abs(e[0]-18) < 2 and abs(e[1]-18) < 2 and e[2] < 6:
        return 'horn'                                    # 25-tooth disc horn, 18 mm across
    if abs(e[0]-8) < 1 and abs(e[1]-5.5) < .6 and abs(e[2]-5.5) < .6:
        return 'm3'                                      # M3 x 8 socket head (a joint stop)
    if abs(e[0]-10) < 1 and abs(e[1]-2.8) < .4 and abs(e[2]-2.8) < .4:
        return 'm3'                                      # M3 drawn as a plain shank (the base lid)
    if abs(e[0]-6.4) < .5 and abs(e[1]-1.9) < .3:
        return 'servo_screw'                             # the servo's own mounting screws
    if (shell.is_watertight and abs(shell.volume)*1e9 > 2000) or e[0] > 50:
        return 'printed'
    return 'servo'


def clusters(shells, gap=.0004):
    """groups of shells whose bounding boxes touch (a servo is drawn as many surface pieces)"""
    parent = list(range(len(shells)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]; i = parent[i]
        return i
    bounds = [s.bounds for s in shells]
    for i in range(len(shells)):
        for k in range(i+1, len(shells)):
            if np.all(bounds[i][0]-gap <= bounds[k][1]) and np.all(bounds[k][0]-gap <= bounds[i][1]):
                parent[find(i)] = find(k)
    groups = {}
    for i, shell in enumerate(shells):
        groups.setdefault(find(i), []).append(shell)
    return list(groups.values())


def split(mesh, servos):
    """every triangle into exactly one piece: connected components, sorted by kind; the largest touching groups of
    servo-like shells are the link's servos, smaller groups near one join it, the rest belong to the printed part"""
    shells = {k: [] for k in ('printed', 'horn', 'servo_screw', 'm3', 'servo')}
    for component in trimesh.graph.connected_components(mesh.face_adjacency, nodes=np.arange(len(mesh.faces)), min_len=1):
        piece = mesh.submesh([np.sort(np.asarray(component))], append=True, repair=False)
        shells[kind(piece)].append(piece)
    parts = {p: [] for p in ('structure', 'motors', 'horns', 'fasteners')}
    for k in ('printed', 'horn', 'servo_screw', 'm3'):
        parts[PART_OF[k]] += shells[k]
    groups = sorted(clusters(shells['servo']), key=lambda g: -sum(x.area for x in g)) if shells['servo'] else []
    seeds = [list(g) for g in groups[:servos]]
    centres = [np.mean([x.centroid for x in g], axis=0) for g in seeds]
    for group in groups[servos:]:
        centre = np.mean([x.centroid for x in group], axis=0)
        near = min(range(len(seeds)), key=lambda i: np.linalg.norm(centres[i]-centre)) if seeds else None
        if near is not None and np.linalg.norm(centres[near]-centre) < .03:
            seeds[near] += group
        else:
            parts['structure'] += group
    for group in seeds:
        parts['motors'] += group
    return {p: v for p, v in parts.items() if v}


def import_reference(raft):
    raft = Path(raft).resolve()
    model = raft/'firmware/tools/sim/model'
    reference = json.loads((model/'raft_reference.json').read_text(encoding='utf-8'))
    if reference.get('schema_version') != 1 or reference.get('robot') != 'raft':
        raise ValueError('Expected a RAFT model reference, schema version 1')
    try:
        commit = subprocess.run(['git', '-C', str(raft), 'rev-parse', 'HEAD'], capture_output=True, text=True,
                                check=True).stdout.strip()
        dirty = bool(subprocess.run(['git', '-C', str(raft), 'status', '--porcelain', '--', 'firmware/tools/sim/model'],
                                    capture_output=True, text=True, check=True).stdout.strip())
    except (OSError, subprocess.CalledProcessError):
        commit, dirty = None, None
    reference['imported_from'] = {'repository': 'https://github.com/Source-Robotics/RAFT-teleoperation-arm',
                                  'commit': commit, 'uncommitted_changes': dirty,
                                  'file': 'firmware/tools/sim/model/raft_reference.json'}
    for directory in ['meshes', 'visual']:
        if (SOURCE/directory).exists():
            shutil.rmtree(SOURCE/directory)
        (SOURCE/directory).mkdir(parents=True)
    partitions = {}
    for link in reference['links']:
        cad = model/'meshes'/link['mesh']['cad_file']
        data = cad.read_bytes()
        if hashlib.sha256(data).hexdigest() != link['mesh']['sha256']:
            raise ValueError(f'{cad.name} does not match the reference: run gen_model.py in the RAFT checkout first')
        (SOURCE/'meshes'/f'{link["name"]}.stl').write_bytes(data)
        mesh = trimesh.load(cad)                            # merges duplicate vertices, keeps every triangle
        parts = split(mesh, SERVOS[link['name']])
        record = {'source': f'meshes/{link["name"]}.stl', 'triangles': int(len(mesh.faces)),
                  'servos': SERVOS[link['name']], 'parts': {}}
        for part, shells in parts.items():
            piece = trimesh.util.concatenate(shells)
            piece.export(SOURCE/'visual'/f'{link["name"]}_{part}.stl')
            record['parts'][part] = {'file': f'visual/{link["name"]}_{part}.stl', 'triangles': int(len(piece.faces))}
        total = sum(p['triangles'] for p in record['parts'].values())
        area = sum(trimesh.util.concatenate(s).area for s in parts.values())
        assert total == record['triangles'] and abs(area-mesh.area) < 1e-9*max(1., mesh.area), (link['name'], total)
        partitions[link['name']] = record
        print(f'{link["name"]:16s} {record["triangles"]:6d} triangles -> '
              + ', '.join(f'{p} {v["triangles"]}' for p, v in record['parts'].items()), flush=True)
    (SOURCE/'visual/partitions.json').write_text(json.dumps(partitions, indent=2)+'\n', encoding='utf-8')
    (SOURCE/'reference.json').write_text(json.dumps(reference, indent=1)+'\n', encoding='utf-8')
    print(f'Imported the RAFT reference from {raft} ({commit or "no git"}{", uncommitted model changes" if dirty else ""}).')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--raft', required=True, type=Path, help='a RAFT-teleoperation-arm checkout')
    import_reference(parser.parse_args().raft)
