# ROS 2: install and display the models

[Back to the model instructions](../README.md)

The four URDF packages provide native `ament_cmake` installation and a ROS 2
`display.launch.py` launcher. Choose a tool or jaw option, then view it in RViz
and move its joints with sliders. Linux CI checks ROS 2 **Humble / Ubuntu 22.04**
and **Jazzy / Ubuntu 24.04**, including installed assets, TF, mimic joints and
the RViz/joint-slider applications.

These launchers display the maintained URDFs. The sliders publish preview joint
states; they do not command motors or simulate dynamics. The existing MuJoCo
packages and raw URDF entry files are unchanged.

For collision-checked motion planning and simulated trajectory execution, add
the matching `par6_moveit_config` or `parol6_moveit_config` package and follow
the [MoveIt 2 guide](https://github.com/Source-Robotics/robot_descriptions/blob/main/docs/MOVEIT2.md).
It supports every mounted tool/jaw option and uses mock controllers.

## Install one downloaded package

Start with ROS 2 installed on its matching Ubuntu version. Use a fresh terminal
and source the appropriate distribution (replace `jazzy` with `humble` below if
needed):

```bash
source /opt/ros/jazzy/setup.bash
sudo apt install python3-colcon-common-extensions python3-rosdep
rosdep update
mkdir -p ~/source_robot_ws/src
unzip ~/Downloads/par6_urdf.zip -d ~/source_robot_ws/src
cd ~/source_robot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select par6_description
source install/setup.bash
ros2 launch par6_description display.launch.py
```

If rosdep has never been initialized on the machine, run `sudo rosdep init`
once before `rosdep update`. The default display is PAR6 with MSG Solid jaws.
Use another URDF ZIP/package name for PAROL6, standalone MSG or standalone SSG48.
You only need the selected package; no CAD tools, authoring virtual environment,
MuJoCo or source-generation build is needed to install it in ROS 2.

## Install from the central checkout instead

After sourcing ROS 2 and installing colcon/rosdep:

```bash
mkdir -p ~/source_robot_ws/src
git clone https://github.com/Source-Robotics/robot_descriptions.git ~/source_robot_ws/src/robot_descriptions
cd ~/source_robot_ws
rosdep install --from-paths src/robot_descriptions/urdf --ignore-src -r -y
colcon build --base-paths src/robot_descriptions/urdf
source install/setup.bash
```

This builds all four packages. Run colcon against the `urdf/` packages; there
is no need to run `tools/build.py` to use the committed models. After opening a
new terminal, source both `/opt/ros/<distro>/setup.bash` and your workspace's
`install/setup.bash` again.

## Choose a robot and tool

Use `par6_description` or `parol6_description` with the same arguments:

| Configuration | Arguments after `display.launch.py` |
| --- | --- |
| Flange only | `end_effector:=flange` |
| MSG / Solid jaws (default) | `end_effector:=msg jaws:=solid` |
| MSG / Soft TPU jaws | `end_effector:=msg jaws:=soft_tpu` |
| SSG48 / Solid jaws | `end_effector:=ssg48 jaws:=solid` |
| SSG48 / Fin-ray jaws | `end_effector:=ssg48 jaws:=fin_ray` |
| Pneumatic gripper | `end_effector:=pneumatic` |

For example:

```bash
ros2 launch par6_description display.launch.py end_effector:=msg jaws:=soft_tpu
ros2 launch parol6_description display.launch.py end_effector:=ssg48 jaws:=fin_ray
```

Run one display at a time. `jaws:=default` selects Solid jaws for either electric
gripper. The historical `end_effector:=msg_tpu_glue` and
`end_effector:=ssg48_stock` spellings also select the existing alternate models.
Incompatible choices are rejected with an explanation, rather than silently
showing a different jaw. Omit `jaws` for flange-only or pneumatic configurations.

## Display a standalone gripper

```bash
ros2 launch msg_description display.launch.py jaws:=solid
ros2 launch msg_description display.launch.py jaws:=soft_tpu
ros2 launch ssg48_description display.launch.py jaws:=solid
ros2 launch ssg48_description display.launch.py jaws:=fin_ray
```

Each launcher opens RViz and a joint-slider window. The two fingers retain
their mimic relation; move the independent finger slider to open or close them.
The gripper-only root frame is `flange`; the arm root is `base_link`. RViz is
configured for the appropriate fixed frame, visual materials and model scale.

## Display options

| Argument | Default | Use |
| --- | --- | --- |
| `gui` | `true` | `false` uses the non-GUI joint-state publisher at its initial pose. |
| `rviz` | `true` | `false` publishes the model/TF without opening RViz. |
| `publish_joint_states` | `true` | `false` uses joint states from your own simulator/controller. |
| `rviz_materials` | `true` | Preserves individual part colors in RViz; see the display compatibility note below. |
| `use_sim_time` | `false` | `true` uses an external simulator's `/clock`. |
| `rviz_config` | Installed `config/display.rviz` | Path to your own RViz configuration. |

For a headless preview:

```bash
ros2 launch par6_description display.launch.py gui:=false rviz:=false
```

To visualize an existing joint-state source:

```bash
ros2 launch par6_description display.launch.py publish_joint_states:=false
```

The selected source must publish matching joint names on `/joint_states`.
The launcher publishes `/robot_description`, `/tf` and `/tf_static`. When using
simulated time, the external source must also supply `/clock`. Stop the launch
with Ctrl+C. RViz displays geometry and frames; the MSG camera frames do not
provide a live camera image or camera driver.

## Assets and compatibility

ROS launch resolves the selected URDF's relative asset references to
`package://<product>_description/...` in memory. Models on disk keep their
portable filenames, physical properties, colors, joint names and frames. Meshes
and the PAR6 badge texture are installed with the package, so a normal colcon
install works without the source directory.

Humble/Jazzy RViz applies one material to multiple visuals on the same URDF
link. The display launcher therefore moves additional visuals to massless
fixed child frames in its in-memory description. Names end in
`_rviz_visual_<n>`; each is attached to the original link by an identity fixed
transform. This preserves separate motor/lens/jaw colors and the textured
badge, while keeping every original joint/frame transform, collision and
inertial definition intact. No model files or meshes on disk are rewritten.
Use `rviz_materials:=false` when another application requires the original
link/joint tree rather than the RViz-compatible display tree.

During arm startup, KDL logs a root-inertia warning for `base_link`. Its TF
tree does not use the fixed root's inertia. The existing base inertial data
is deliberately retained in the URDF; the warning does not change joint/frame
transforms. This viewer is not computing robot dynamics.

ROS 1 users retain `display.launch` and the conditional catkin build path when
working in a sourced ROS 1 environment. ROS 2 users should use
`display.launch.py`. The older `gazebo.launch` files are ROS 1 scaffolds;
this release does not add Gazebo or a hardware-control driver. ROS 2 source
installation, ament conventions and robot-state publication are documented by
[ROS 2](https://docs.ros.org/en/jazzy/index.html) and
[robot_state_publisher](https://github.com/ros/robot_state_publisher).
