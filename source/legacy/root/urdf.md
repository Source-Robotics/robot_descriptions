# PAR6 ROS 1 / URDF packages

The `URDF/` directory contains three Catkin packages:

| Package | URDF |
| --- | --- |
| `par6_flange` | `urdf/par6_flange.urdf` |
| `par6_msg_gripper` | `urdf/PAR6_MSG.urdf` |
| `par6_ssg48_gripper` | `urdf/par6_ssg48_urdf.urdf` |

## Use in a Catkin workspace

Copy the package you need into a ROS 1 Catkin workspace, then build and source
the workspace:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Display a model in RViz:

```bash
roslaunch par6_msg_gripper display.launch
roslaunch par6_ssg48_gripper display.launch
roslaunch par6_flange display.launch
```

Spawn a model in Gazebo:

```bash
roslaunch par6_msg_gripper gazebo.launch
```

## Important limitation

The generated URDF files still contain zero placeholders for joint position,
velocity, and effort limits. As a result, a viewer may show a valid model while
joint-state controls remain fixed. These placeholders must be replaced with
authoritative PAR6 hardware limits before using the URDF for planning, control,
or safety checks.

The second jaw is a mimic joint in both gripper packages and is intentionally
not listed as a separately controlled joint.

## Browser viewers

Browser viewers can be useful for geometry inspection, but support for ROS
`package://` paths and mimic joints differs:

- <https://gkjohnson.github.io/urdf-loaders/javascript/example/bundle/>
- <https://urdf.d-robotics.cc/>
- <https://viewer.robotsfan.com/>

Upload the complete individual Catkin package so its `meshes/` directory remains
available.

## References

- [SolidWorks URDF export guide](https://www.youtube.com/watch?v=I08lO_SRBbk)
- [URDF to MuJoCo overview](https://www.youtube.com/watch?v=v2OfmQaoIH4)
