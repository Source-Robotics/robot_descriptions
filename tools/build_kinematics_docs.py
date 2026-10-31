"""Publish normalized PAR6/PAROL6 kinematics and actuation documentation."""

from html import escape
import json
import xml.etree.ElementTree as ET

import numpy as np

from parol6_model import (ROOT, URDF, MJCF, ARM_JOINTS, ARM_LINKS, urdf_fk,
                          FOREARM_DISTANCE, FLANGE_DISTANCE, PAR6_FLANGE_DISTANCE)


def diagram():
    sections = ['''<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="650" viewBox="0 0 1200 650">
<rect width="1200" height="650" fill="#f7f9fc"/>
<style>text{font-family:Arial,sans-serif;fill:#172c40}.title{font-size:25px;font-weight:bold}.label{font-size:16px}.small{font-size:14px}.mono{font-family:monospace;font-size:15px}</style>
<defs><marker id="arrow" markerWidth="6" markerHeight="6" refX="5" refY="3" orient="auto-start-reverse"><path d="M0 0L6 3L0 6Z" fill="#16828f"/></marker></defs>
<text x="34" y="39" class="title">PAR6 / PAROL6: common axes and wrist-frame convention</text>
<text x="34" y="67" class="label">Actual packaged zero-pose joint centers, side projection in X-Z. Joint 5 and joint 6 share a center.</text>
''']
    for column, (label, path) in enumerate([
        ('PAR6', ROOT/'urdf/par6_description/par6_flange.urdf'),
        ('PAROL6 / standardized frames', URDF/'parol6_flange.urdf')]):
        xbase = 40+column*600
        frames, tree = urdf_fk(path), ET.parse(path)
        points = [frames[name][:3, 3] for name in ARM_LINKS]
        def screen(point):
            return xbase+52+point[0]*1420, 465-point[2]*820
        sections.append(f'<rect x="{xbase-12}" y="89" width="565" height="462" rx="12" fill="white" stroke="#d7e0eb"/>')
        sections.append(f'<text x="{xbase+9}" y="121" class="title">{label}</text>')
        points_text = ' '.join(f'{screen(p)[0]:.3f},{screen(p)[1]:.3f}' for p in points)
        sections.append(f'<polyline points="{points_text}" fill="none" stroke="#6f8092" stroke-width="9" stroke-linejoin="round"/>')
        fx, fy = screen(frames['flange'][:3, 3]);x, y = screen(points[-1])
        sections.append(f'<line x1="{x}" y1="{y}" x2="{fx}" y2="{fy}" stroke="#b08730" stroke-width="6"/>')
        sections.append(f'<rect x="{fx-4}" y="{fy-10}" width="8" height="20" fill="#b08730"/><text x="{fx-18}" y="{fy-20}" class="small">Flange</text>')
        for i, (name, link, point) in enumerate(zip(ARM_JOINTS, ARM_LINKS, points), start=1):
            if i == 6:
                continue
            x, y = screen(point)
            sections.append(f'<circle cx="{x}" cy="{y}" r="7" fill="#244764" stroke="white" stroke-width="2"/>')
            dx, dy = {1: (-26, 24), 2: (-34, -18), 3: (-34, 6), 4: (-34, -10), 5: (-10, -37)}[i]
            sections.append(f'<text x="{x+dx}" y="{y+dy}" class="label">{("J5/J6" if i == 5 else f"J{i}")}</text>')
            if i <= 4:
                axis = np.fromstring(tree.find(f'joint[@name="{name}"]/axis').get('xyz'), sep=' ')
                direction = frames[link][:3, :3]@axis
                if abs(direction[1]) > .9:
                    cx, cy = x+18, y+17
                    sections.append(f'<circle cx="{cx}" cy="{cy}" r="7" fill="white" stroke="#16828f" stroke-width="2"/>')
                    if direction[1] < 0:
                        sections.append(f'<circle cx="{cx}" cy="{cy}" r="2.5" fill="#16828f"/>')
                    else:
                        sections.append(f'<path d="M{cx-4} {cy-4}L{cx+4} {cy+4}M{cx-4} {cy+4}L{cx+4} {cy-4}" stroke="#16828f" stroke-width="1.5"/>')
                else:
                    sections.append(f'<line x1="{x}" y1="{y}" x2="{x+direction[0]*34}" y2="{y-direction[2]*34}" stroke="#16828f" stroke-width="2" marker-end="url(#arrow)"/>')
        length = f'{FOREARM_DISTANCE*1000:.2f}'
        sections.append(f'<text x="{xbase+155}" y="217" class="small">Forearm axis distance: {length} mm</text>')
        sections.append(f'<text x="{xbase+9}" y="510" class="small">Local joint axes, J1 to J6:</text><text x="{xbase+9}" y="533" class="mono">{escape("+Z, +Z, +Z, +Z, +Z, -Z")}</text>')
    sections.append('''<text x="34" y="583" class="label">Same public arm frames and 177 mm forearm. The intended DH difference is the wrist-to-flange d6.</text>
<text x="34" y="610" class="label">Flange reach: PAR6 57.1 mm; PAROL6 50.5 mm. Tool spacing follows each wrist housing.</text>
<text x="34" y="636" class="small">Teal marks show positive directions; dot = -Y, cross = +Y. Historical export conversion is documented separately.</text></svg>''')
    return '\n'.join(sections)


def actuation_document():
    root = ET.parse(MJCF/'parol6_flange.xml')
    servo = root.find('default/default[@class="Y"]/position')
    joint = root.find('default/default[@class="Y"]/joint')
    return f'''# Position actuation and open-loop hardware

Open-loop stepper control means that commanded step counts are not corrected
using measured joint position. It does not imply infinite mechanical stiffness.
Real steppers have finite torque and load-angle deflection, and can lose steps
or stall. A position servo in a simulator is an approximation of commanded
position tracking, not evidence that the real arm has a position PID loop.

## Current finite simulation settings

The six arm position actuators retain the PAR6 simulation estimates:

| Parameter | Current value |
| --- | --- |
| Position gain Kp | `{servo.get('kp')}` Nm/rad |
| Explicit actuator Kv | `{servo.get('kv', '0')}` Nms/rad |
| Passive joint damping | `{joint.get('damping')}` Nms/rad |
| Passive joint friction loss | `{joint.get('frictionloss')}` Nm |
| Actuator torque range | `{servo.get('forcerange')}` Nm |
| Integrator | `implicitfast` |

These gains and torque limits are uncalibrated estimates, not measured PAROL6
motor specifications. Gripper actuators keep their separate existing settings.
No gains were increased solely because PAROL6 uses open-loop hardware.
For PAROL6 the user selected finite position control with torque limits as
the default.

For stiff, force-responsive simulation, tune finite Kp and actuator Kv per
joint, retain finite torque limits and test the timestep, loads and contacts.
A useful single-joint starting relation is Kv = 2*zeta*sqrt(Kp*J_eff), allowing
for existing passive damping; coupled-arm inertia varies with configuration.
Excessively large Kp is not a substitute for this check. MuJoCo 3.3 position
actuators support kp/kv and torque clamping; its documentation recommends an
implicit integrator when using actuator damping.

For exact commanded motion in a viewer, set the joint positions and call
`mj_forward` instead of advancing the arm with `mj_step`. This is kinematic
replay: its arm motion is prescribed and cannot establish realistic motor
torques or contact response. It is a separate application/viewer behavior.

The separate PAR6 motion application's jog tool already defaults to kinematic
placement when `--physics` is omitted. Its optional `--physics stiff` preset
overrides gains at runtime; it is not a calibrated PAROL6 actuator model or
a change to these package files. Model selection in that application is
separate from the frame convention and package actuation described here.

## URDF

Standard URDF describes joint structure, inertias, limits and optional passive
damping/friction. Position PID gains belong in the selected simulator/controller
configuration, for example a ROS controller YAML file. A position-command
interface may prescribe positions directly, or a simulator adapter may implement
a finite servo. Therefore a URDF alone cannot promise a particular stiffness.
Large URDF damping only resists velocity; it does not create perfect tracking.

Sources: [MuJoCo 3.3 actuation](https://mujoco.readthedocs.io/en/3.3.0/XMLreference.html#actuator-position),
[ROS controller parameters](https://control.ros.org/rolling/doc/ros2_controllers/joint_trajectory_controller/doc/parameters.html),
[TI stepper feedback/stall reference](https://www.ti.com/lit/ug/tiducy6/tiducy6.pdf).
'''


def build(report=None):
    if report is None:
        report = json.loads((ROOT/'reports/parol6_validation.json').read_text())
    maximum = lambda key: [max(e[key][i] for e in report['profiles'].values()) for i in range(2)]
    source_error = maximum('corrected_source_arm_fk_max_mm_deg')
    residual = maximum('canonical_chain_residual_max_mm_deg')
    flange_error = maximum('mapped_par6_flange_max_mm_deg')
    format_error = maximum('urdf_native_tcp_max_mm_deg')
    ideal = report['ideal_dh_error_max_mm_deg']
    text = f'''# PAR6 and PAROL6: common DH and joint convention

The generated PAROL6 packages now use **PAR6 joint directions and origin
orientations, including J6**, per user instruction. Public package angles
are shared: `q_PAROL6 = q_PAR6`. Both use the corrected **177 mm forearm**.
The intended difference in the flange-ending standard DH tables is now
**d6**, reflecting the shorter PAROL6 wrist housing.

![Joint-center and frame diagram](kinematics_comparison.svg)

## Standard DH table

`A_i = Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i)`; endpoint is the common
physical flange. Distances are millimetres; alpha is displayed in degrees.
Theta expressions use radians, with `q` the package angles in radians.

| Joint | theta, both robots | alpha, both | a, both | PAR6 d | PAROL6 d |
| ---: | --- | ---: | ---: | ---: | ---: |
| 1 | `q1` | -90 | 23.420721 | 110.5 | 110.5 |
| 2 | `q2 - pi/2` | 180 | 180 | 0 | 0 |
| 3 | `q3 + pi` | 90 | -43.5 | 0 | 0 |
| 4 | `q4` | -90 | 0 | -177.00 | -177.00 |
| 5 | `q5` | 90 | 0 | 0 | 0 |
| 6 | `q6 + pi` | 180 | 0 | **-57.1** | **-50.5** |

## Geometry corrections and mounting clearance

The user corrected PAROL6's forearm from the historical export's 176.35 mm
to 177.00 mm. Its J5 origin is now `(0, 0, -0.177)` m relative to wrist_1_link.
The lower-arm visual and collision origins both move `(0, 0, -0.00065)` m
in that same frame, keeping the housing geometry aligned with the wrist.
The original STL records are preserved; no vertices were stretched.

The included wrist housing surfaces end at +41.1 mm (PAR6) and +34.5 mm
(PAROL6), measured along local +Y of wrist_2_link, which follows J6's
forward axis. These are the largest connected housing components; internal
motor/shaft components do not define the housing mounting face.
Keeping PAR6's approximately 1 mm housing-to-flange rear-face clearance gives:

`PAROL6 flange reach = 57.1 + (34.5 - 41.1) = 50.5 mm`.

This is a CAD-derived spacing match requested by the user, not an independent
hardware measurement. The entire flange subtree moves 6.6 mm toward J6.
Every flange-relative tool, camera, finger, collision, inertial and TCP
transform stays unchanged. Tool meshes retain their original part coordinates.
Gripper TCP axial reach is consequently 134.3 mm on PAROL6 versus 140.9 mm
on PAR6; the MSG lateral TCP offset remains 28.35 mm on both.

The shorter tool lever arm affects generalized dynamics even with inherited
local mass/COM/inertia values. These values and finite actuator settings were
not retuned or re-estimated by the geometry correction.
The existing RCB reference rounds the shoulder offset to 23.420 mm; both CAD
descriptions use 23.420721 mm. This is a reference-data precision difference,
not a PAR6/PAROL6 geometry difference.

## Matching public frames

Both models use `base_joint`, `shoulder_joint`, `elbow_joint`,
`wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint`, in that order. Local axes
are `+Z, +Z, +Z, +Z, +Z, -Z`. J6's URDF origin RPY is `(pi/2, 0, pi)`
in both models; each native model inherits the same PAR6 quaternion.
The wrist_3_link-to-flange transform has identity rotation and local XYZ
`(0, 0, +0.0571)` m on PAR6, `(0, 0, +0.0505)` m on PAROL6.
All flange-relative tools, cameras,
finger coupling, contacts and TCP definitions are unchanged.

Source PAROL6 joint types/physical bounds are retained with bounds converted
to the normalized coordinates; for example old L3 `[-2, 1.3]` becomes
elbow_joint `[-1.3, 2]`. The first five joints remain revolute and J6 remains
continuous. Existing PAR6 packages use continuous arm joints. Joint types,
limits, arm meshes and actuation assumptions are separate from a DH table.

## Historical export conversion

The original supplied SolidWorks export remains byte-for-byte preserved at
`source/parol6/original/` in the authoring repository. It is not the public
angle convention of the generated packages. To compare its old coordinates:

```text
q_source_L1_to_L6 = [q1, q2, -q3, -q4, -q5, -q6]
```

Old L1-L6 names map in order to the six descriptive link/joint names. L1-L5
CAD part coordinates still hold the corresponding meshes and local inertias.
Old L6 is rotated by 180 degrees about local Y relative to normalized
wrist_3_link; its old end-tool mesh is retained as source only and replaced
by the selected shared tool. This frame change therefore requires no rotation
of the shared tools' own meshes or inertias. Source four-decimal RPY values
are replaced in generated joints by the PAR6 reference orientations.

The axis standardization replaced the earlier 2.9.0 package convention. Consumers
of that local version must apply the historical sign conversion once when
migrating saved joint-angle trajectories. New PAR6/PAROL6 trajectories share
coordinates directly. The existing PAR6 DH/hardware convention remains
`TH = q + [0, -pi/2, pi, 0, 0, pi]`. An export or standardized model does
not independently verify a physical PAROL6 controller's encoder/homing mapping.

## Verification

Every profile was checked at {report['poses_per_profile']} configurations.
The maximum residual
against matching PAR6 frames is **{residual[0]:.3g} mm /
{residual[1]:.3g} degrees**. At the same public angles the physical flange
differs by at most **{flange_error[0]:.6f} mm /
{flange_error[1]:.3g} degrees**. The expected translation magnitude is 6.6 mm
along the wrist's local axial direction; an explicit vector check verifies it.

After applying the intentional +0.65 mm forearm correction to the sign-mapped
historical export, its arm comparison is within
**{source_error[0]:.6f} mm / {source_error[1]:.6f} degrees**; the residue comes
from replacing its rounded joint-origin angles with the PAR6 orientations.
The raw historical-source discrepancy is recorded separately in the JSON.
Native/URDF TCP agreement is within **{format_error[0]:.6f} mm /
{format_error[1]:.6f} degrees**. Maximum ideal-DH approximation error across
both formats is **{ideal[0]:.6f} mm / {ideal[1]:.6f} degrees**; the existing
PAR6 native quaternion precision is inherited, and its URDF uses full precision.

Mass/COM/central inertia, native armature and complete native tool subtrees
are checked against PAR6 for all six profiles. Original arm collision meshes
and visual triangle records are preserved. Available tools: flange; MSG Solid
jaws (default) or Soft TPU jaws; SSG48 Solid jaws (default) or Fin-ray jaws;
and the original side-gripping pneumatic assembly. The pneumatic jaws use
coupled 3.5 mm slides; its legacy 195 g total and CAD-derived mass distribution
are unverified estimates. Its operational TCP has 45.25 mm lateral offset and
68.9 / 62.3 mm axial reach on PAR6 / PAROL6. See `PNEUMATIC.md`.

The shorter PAROL6 flange reference is derived from the supplied housing
geometry. The original export has no independently named flange marker.
See `DYNAMICS.md`, `DYNAMICS_AUDIT.md`, and `ACTUATION.md` for inherited physical
data, finite servo settings and calibration limits. Machine-readable results:
`reports/parol6_validation.json` in the authoring repository.
'''
    svg, actuation = diagram(), actuation_document()
    for directory in [ROOT/'mujoco/par6', ROOT/'urdf/par6_description', MJCF, URDF]:
        (directory/'KINEMATICS_COMPARISON.md').write_text(text, encoding='utf-8')
        (directory/'kinematics_comparison.svg').write_text(svg, encoding='utf-8')
        (directory/'ACTUATION.md').write_text(actuation, encoding='utf-8')
    (ROOT/'reports/par6_parol6_kinematics.md').write_text(text, encoding='utf-8')
    (ROOT/'reports/kinematics_comparison.svg').write_text(svg, encoding='utf-8')
    dh = '''# PAROL6 DH and joint convention

The generated package uses PAR6 axes and joint-origin orientations, including
J6. Public joint coordinates and the 177 mm forearm match PAR6 directly.
Standard-DH d4 is -177.00 mm on both. Only flange-ending d6 differs:
PAROL6 -50.5 mm, PAR6 -57.1 mm. See the
[complete comparison](KINEMATICS_COMPARISON.md) for the numeric table,
historical-export conversion and validation. The wrist-to-flange offset is
+50.5 mm on local Z with identity rotation. See `ACTUATION.md` for controller
behavior; open-loop hardware does not imply infinite simulated stiffness.
'''
    audit = ['# PAROL6 inheritance and normalized-frame validation', '',
             'Local physical properties are inherited from PAR6 by instruction. Public arm frames and the 177 mm forearm match PAR6. PAROL6 flange reach is 50.5 mm versus PAR6 57.1 mm, matching the shorter housing.', '',
             '| Profile | Canonical-chain residual (mm) | Native/URDF TCP (mm) | M difference versus PAR6 (relative) | Gravity difference versus PAR6 (Nm) |',
             '| --- | ---: | ---: | ---: | ---: |']
    for name, entry in report['profiles'].items():
        audit.append(f"| {name} | {entry['canonical_chain_residual_max_mm_deg'][0]:.3g} | {entry['urdf_native_tcp_max_mm_deg'][0]:.6f} | {entry['mapped_par6_mass_matrix_max_relative_difference']:.6g} | {entry['mapped_par6_gravity_max_difference_nm']:.6g} |")
    audit += ['', f"Each profile uses {report['poses_per_profile']} configurations. Generalized dynamics differences include the 6.6 mm shorter tool lever arm. Local mass/COM/tensors and gains remain inherited estimates.", '',
              'See `KINEMATICS_COMPARISON.md`, `DYNAMICS.md`, and `ACTUATION.md` for assumptions and calibration status.', '']
    for directory in [MJCF, URDF]:
        (directory/'DH.md').write_text(dh, encoding='utf-8')
        (directory/'DYNAMICS_AUDIT.md').write_text('\n'.join(audit), encoding='utf-8')


if __name__ == '__main__':
    build()
