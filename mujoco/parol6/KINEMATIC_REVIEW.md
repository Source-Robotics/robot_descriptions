# PAROL6 kinematic review sheets

All six profiles use the same orthographic scale and zero-angle pose. Rendered meshes come from MuJoCo; dimensions and fixed TCP transforms come from the corresponding URDF. Each JSON contains source hashes, the complete fixed transform and independent FK-check results.

| Configuration | Diagram | Raster | Numeric data |
| --- | --- | --- | --- |
| Flange only | [SVG](kinematics_review/flange.svg) | [PNG](kinematics_review/flange.png) | [JSON](kinematics_review/flange.json) |
| MSG / Solid jaws | [SVG](kinematics_review/msg.svg) | [PNG](kinematics_review/msg.png) | [JSON](kinematics_review/msg.json) |
| MSG / Soft TPU jaws | [SVG](kinematics_review/msg_tpu_glue.svg) | [PNG](kinematics_review/msg_tpu_glue.png) | [JSON](kinematics_review/msg_tpu_glue.json) |
| SSG48 / Solid jaws | [SVG](kinematics_review/ssg48.svg) | [PNG](kinematics_review/ssg48.png) | [JSON](kinematics_review/ssg48.json) |
| SSG48 / Fin-ray jaws | [SVG](kinematics_review/ssg48_stock.svg) | [PNG](kinematics_review/ssg48_stock.png) | [JSON](kinematics_review/ssg48_stock.json) |
| Pneumatic gripper | [SVG](kinematics_review/pneumatic.svg) | [PNG](kinematics_review/pneumatic.png) | [JSON](kinematics_review/pneumatic.json) |

Standard DH: `Rz(theta) Tz(d) Tx(a) Rx(alpha)`. Joint values `q` use package coordinates; the theta offsets are already in the table. Apply them once. Alpha is in degrees; theta uses radians. `a` is a translation along X; `d` is a translation along Z.

The last row ends at the **configured operational TCP**. It is not a measured jaw contact center. Each gripper keeps the same TCP across its jaw options. In particular, SSG48 Solid jaws are about 23.4 mm shorter than Fin-ray jaws. See [GRASP_FRAMES.md](GRASP_FRAMES.md).

The pneumatic profile retains the legacy operational TCP with 45.25 mm lateral offset and 68.9 / 62.3 mm axial reach on PAR6 / PAROL6. Its original side-gripping jaws move symmetrically; mass/actuator values remain estimates. See [PNEUMATIC.md](PNEUMATIC.md).

PAR6/PAROL6 share joint-angle conventions and a 177 mm forearm (`d4 = -177 mm`). The intended flange-table difference is `d6 = -57.1 / -50.5 mm`, matching the different wrist housings. Small additional differences in displayed precision are inherited from individual URDF exports, and the numerical checks measure their effect. See [KINEMATICS_COMPARISON.md](KINEMATICS_COMPARISON.md).

Zero pose is a wrist singularity and is used here only as a clear dimensioned reference. Joint-center labels are physical joint centers, not a drawing of every intermediate DH frame. J5 and J6 centers coincide. The side projection can hide overlapping jaw surfaces.

Regenerate in the authoring checkout with `python tools/build_review_sheets.py`. Add `--render` to refresh snapshots after visual changes and `--pdf` to assemble the optional booklet.
