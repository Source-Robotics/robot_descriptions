# Authoring inputs

[Repository map](../REPOSITORY_GUIDE.md) · [Build instructions](../docs/maintenance/BUILDING.md)

Use this folder when changing the models. Most users only need a released
package from `mujoco/` or `urdf/`.

| Folder | What belongs here |
| --- | --- |
| [appearance/](appearance/) | Shared `colors.json` palette and per-part overrides. |
| [artwork/](artwork/) | Editable SVG badge, placement settings and checked texture cache. |
| [dynamics/](dynamics/) | Measured masses, captured RCB data and frame/property mappings. |
| [jaws/](jaws/) | STEP jaw sources, build settings and checked tessellation caches. |
| [parol6/](parol6/) | Supplied original export and normalization/material policy. |
| [raft/](raft/README.md) | RAFT's model reference, CAD meshes, visual pieces, documents and previews, imported from the RAFT repository. |
| [pneumatic/](pneumatic/) | Pneumatic configuration and checked convex contact pieces. |
| [grippers/](grippers/) | Canonical standalone-gripper preview images. |
| [reviews/](reviews/) | Checked robot snapshots used by dimensioned review sheets. |
| [legacy/](legacy/) | Original CAD/URDF exports still used as generation inputs and preservation evidence. |

Edit the JSON/SVG/STEP input appropriate to your change, then rebuild. Generated
check results belong in [reports/](../reports/README.md), not beside build code.
Keep the original exports and checked caches: they support reproducible builds.

`COLCON_IGNORE` and `CATKIN_IGNORE` exclude historical ROS exports from package
discovery. Maintained ROS packages live under `urdf/` and `moveit/`.
