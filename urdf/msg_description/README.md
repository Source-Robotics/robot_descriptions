# Source Robotics MSG: standalone URDF gripper

This self-contained package contains **only the MSG gripper**, with both jaw
options, its fixed mounting frame and configured TCP. All required meshes and
materials are included. Keep the whole folder together when moving or uploading it.

![MSG with Solid jaws](previews/msg_solid.png)

| Jaws | URDF entry file |
| --- | --- |
| Solid jaws (default) | `msg_solid.urdf` |
| Soft TPU jaws | `msg_soft_tpu.urdf` |

The default is Solid jaws. The URDF filenames are at the package root and use relative mesh paths.

Detailed notes are in [docs/](docs/), model images in [previews/](previews/),
and geometry in `meshes/`. The model entry files
remain at the package root.

## Open the model

Open the selected URDF in a viewer with the meshes directory alongside it. For ROS 1, place this package in catkin_ws/src, build and source the workspace, then run:

```bash
roslaunch msg_description display.launch jaws:=solid
```

## Mount and finger controls

The URDF root link, `flange`, is the mounting face. Attach it to your robot
with a fixed joint. `left_finger_joint` drives the mechanism and
`right_finger_joint` follows its retained mimic relation. Your URDF consumer
must honor the mimic joint and exclude adjacent housing/finger collisions.
The supplied display launch is ROS 1; ROS 2 users can load the same URDF using
their own `robot_state_publisher`/RViz launch setup. URDF itself does not
implement the native MuJoCo position servo or contact solver.

Each finger travels 52.5 mm. Both formats preserve their
corresponding arm-mounted tool's joint origins, axes, limits, coupling, collision
surfaces, materials, mass properties and camera definition.
No arm, flange hardware, grasp object, table, mocap target or free joint is in
the gripper model itself. The optional viewer scene adds only lighting and floor.
`flange` is a massless frame; the 55.5 g robot flange is not part of this package.

## Physical properties and current limits

Complete gripper mass: **380 g**. Fixed assembly:
**301.4 g**; each complete moving assembly: **39.3 g**.
Tool COM and inertia remain the current unverified estimates. The two jaw
options share those measured mass targets; TPU/Fin-ray deformation is not
simulated. Actuation parameters and grip force are not hardware calibration.

The retained `tcp` is the operational control frame, not a newly measured
jaw-specific grasp center. Frame and inertia definitions are copied from the
maintained robot assembly; exporting does not infer new physical measurements.

See [validation notes](docs/VALIDATION.md) for the extraction checks and `LICENSE` for CC BY 4.0.

## License and attribution

Models and assets: **CC BY 4.0**. See [attribution and license scope](docs/ATTRIBUTION.md).

[Changing colors](docs/COLORS.md): shared palette, part overrides and illustrated examples.

[MSG built-in camera](docs/MSG_CAMERA.md): simulated views with both jaw options and a rendering example.
