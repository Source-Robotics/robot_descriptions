# RAFT model inputs

[Source index](../README.md) · [Build instructions](../../docs/maintenance/BUILDING.md)

RAFT's packages (`mujoco/raft`, `urdf/raft_description`) are built from this folder by
[`tools/build_raft.py`](../../tools/build_raft.py) and checked by [`tools/verify_raft.py`](../../tools/verify_raft.py).

| File or folder | What it is |
| --- | --- |
| `reference.json` | RAFT's model reference: frames, mass properties, joint ranges and their sources, keyframes, actuation estimates, the J2 spring sites, and 40 sample poses with RAFT's own forward kinematics and gravity torques for the checks. |
| `meshes/` | The CAD mesh of every link, in its link's mesh frame (the complete meshes are the collision geometry). |
| `visual/` | Every link mesh split into printed parts, servos, horns and screws, and `partitions.json` with their triangle counts. Each CAD triangle is in exactly one piece. |
| `docs/` | The package documents; the build fills in the tables from `reference.json`. |
| `README.mujoco.md`, `README.urdf.md` | The two packages' front pages. |
| `previews/` | The package preview (`raft.png`) and the overview picture (`raft_isometric.png`). |

## Where the numbers come from

RAFT's firmware generator, `firmware/tools/gen_model.py` in the
[RAFT repository](https://github.com/Source-Robotics/RAFT-teleoperation-arm), computes the arm's model from its DH
table, the CAD mass properties and the measured joint stops; the same run writes the firmware's own model. It exports
`firmware/tools/sim/model/raft_reference.json` next to the CAD meshes. After a change there, import it again:

```bash
python tools/import_raft_reference.py --raft ../RAFT-teleoperation-arm
python tools/build.py --models-only
```

The import checks the meshes against the reference's hashes, records the RAFT commit in `reference.json`, and splits
the meshes into their visual pieces again. The normal build needs no RAFT checkout.

Previews need OpenGL: `python tools/render_raft_previews.py` renders both pictures after an appearance change.
