# MoveIt 2: plan and preview robot movements

The `par6_moveit_config` and `parol6_moveit_config` packages add motion planning
and simulated trajectory execution to the maintained URDF packages. Each
supports flange-only, both MSG jaws, both SSG48 jaws and the pneumatic gripper.
ROS 2 Humble / Ubuntu 22.04 and Jazzy / Ubuntu 24.04 are the tested targets.

Execution uses **mock hardware**: joint commands are followed ideally. This
demonstrates ROS planning/control integration; it does not simulate motor
dynamics, payload deflection, grasp/contact forces or communicate with RCB.
Use the MuJoCo packages for the existing contact/physics demonstrations.

## Install from downloads

Start with ROS 2 installed on its matching Ubuntu version. In a fresh terminal,
source that distribution (`humble` can replace `jazzy` below):

```bash
source /opt/ros/jazzy/setup.bash
sudo apt install python3-colcon-common-extensions python3-rosdep
# On a machine that has never used rosdep: sudo rosdep init
rosdep update
mkdir -p ~/source_robot_ws/src
unzip ~/Downloads/par6_urdf.zip -d ~/source_robot_ws/src
unzip ~/Downloads/par6_moveit_config.zip -d ~/source_robot_ws/src
cd ~/source_robot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to par6_moveit_config
source install/setup.bash
ros2 launch par6_moveit_config demo.launch.py
```

Download **both ZIPs from the same release**. The MoveIt package contains
configuration/support code and uses the description package's meshes directly.
For PAROL6, use `parol6_urdf.zip`, `parol6_moveit_config.zip` and
`parol6_moveit_config` in the commands above.

Download links:

- [PAR6 URDF](https://github.com/Source-Robotics/robot_descriptions/releases/latest/download/par6_urdf.zip)
  and [PAR6 MoveIt](https://github.com/Source-Robotics/robot_descriptions/releases/latest/download/par6_moveit_config.zip).
- [PAROL6 URDF](https://github.com/Source-Robotics/robot_descriptions/releases/latest/download/parol6_urdf.zip)
  and [PAROL6 MoveIt](https://github.com/Source-Robotics/robot_descriptions/releases/latest/download/parol6_moveit_config.zip).

## Install from a source checkout

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/source_robot_ws/src
cd ~/source_robot_ws/src
git clone https://github.com/Source-Robotics/robot_descriptions.git
cd ~/source_robot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to par6_moveit_config parol6_moveit_config
source install/setup.bash
ros2 launch par6_moveit_config demo.launch.py
```

No CAD regeneration or authoring Python environment is needed. Whole-checkout
colcon discovery finds four description packages and two MoveIt packages;
preserved historical exports under `source/` are ignored.

## Choose a robot, gripper and jaws

```bash
ros2 launch par6_moveit_config demo.launch.py end_effector:=flange
ros2 launch par6_moveit_config demo.launch.py end_effector:=msg jaws:=solid
ros2 launch par6_moveit_config demo.launch.py end_effector:=msg jaws:=soft_tpu
ros2 launch parol6_moveit_config demo.launch.py end_effector:=ssg48 jaws:=solid
ros2 launch parol6_moveit_config demo.launch.py end_effector:=ssg48 jaws:=fin_ray
ros2 launch parol6_moveit_config demo.launch.py end_effector:=pneumatic
```

Either robot supports every row. The default is MSG Solid jaws. Existing aliases
`msg_tpu_glue` and `ssg48_stock` remain accepted. Incompatible jaw/tool choices
fail with a clear error. Omit `jaws` for flange-only and pneumatic.

Run **one demo at a time**, and stop any earlier `display.launch.py`: the demo
starts its own robot state publisher, controllers, `/joint_states`, MoveIt and
RViz. It needs no external simulator, `/clock` or joint sliders.

## Plan and execute in RViz

1. Open the **MotionPlanning** panel and choose the **arm** planning group.
   Its tip is the selected tool's **tcp** frame; changing tools selects the
   matching TCP and collision geometry automatically.
2. Set **Start State** to the current state. Choose the named **ready** goal,
   or move/rotate the interactive marker at the TCP.
3. Click **Plan**. Inspect the preview trajectory, then click **Execute**.
   The mock controllers move the displayed robot along that trajectory.
4. Select the **gripper** group and choose **open** or **closed**, then plan
   and execute. Only the independent finger is commanded; the opposing finger
   follows its URDF mimic relation. Flange-only has no gripper group/controller.
5. Return the arm to its named **home** pose when desired.

The translucent orange robot shows the requested goal pose. The solid robot
shows the current mock-controller state, using the model's normal colors.

Pneumatic hardware is endpoint-only. Its `open` and `closed` presets demonstrate
the two endpoints; the mock trajectory interpolates between them and is not a
binary valve/controller implementation. Intermediate RViz targets are preview
positions, not hardware commands. TPU/Fin-ray geometry remains rigid.

The MSG `closed` preset stops **0.5 mm per finger before the mathematical
lower limit**. The existing mesh at that limit has about 0.8 mm total opposing
jaw overlap. The preset keeps empty-jaw collision checking enabled; the saved
model limit and actual contact/physics model are unchanged.
SSG48's `closed` preset similarly stops **0.1 mm per finger** before its exact
face-contact endpoint. Pneumatic uses its actual endpoints: `open` = 0 and
`closed` = 0.0035 m per finger.

## Joint limits and collision checking

The historical PAR6 URDF has continuous arm joints with zero speed/effort
placeholders. For this planning instance only, those joints become bounded
revolute joints using the **captured RCB software limits**, converted into the
package's angle convention by subtracting `rcb_from_mj_offset_rad`. PAROL6
keeps its own maintained URDF ranges; its stored finite J6 range is also
enforced for planning. No joint origins, axes, COM, masses, inertia or collision
shapes are changed. The raw URDFs and MuJoCo models remain unchanged.

`config/profiles/*.json` records the exact position ranges and their source.
`config/joint_limits.yaml` supplies editable **demo defaults**, currently
0.5 rad/s and 1 rad/s² for the arm, 0.03 m/s and 0.1 m/s² per finger. These
rates and effort placeholders are not measured hardware capabilities. MoveIt
applies trajectory scaling in addition to these maxima.
Positive URDF effort values are retained; the effort defaults only fill the
historical zero placeholders. Mock controllers use position commands.

SRDF collision exemptions cover links in the **same rigid assembly** and
directly connected moving assemblies, where bearing/mounting geometry meets.
Nonadjacent arm links and opposing fingers remain collision checked. No
sampling-based “never collides” shortcuts disable distant link pairs.
The initial planning scene has no workbench or floor collision object: the
RViz grid is visual. Add the actual environment through MoveIt's planning scene
before expecting obstacle avoidance around it.

The ROS display material adapter is enabled, preserving individual motor/jaw
colors using massless fixed visual children. These have no collisions or
inertia. `rviz_materials:=false` omits them if a consumer needs the original
physical link tree; multiple visuals may then share their first material in
RViz. A fixed `world` → `base_link` transform locates the standalone arm.

## Headless use and integration

```bash
ros2 launch par6_moveit_config demo.launch.py end_effector:=ssg48 jaws:=solid rviz:=false
```

The same stack exposes MoveIt's planning, IK/FK, state-validity and trajectory
execution interfaces. The arm controller accepts
`/arm_controller/follow_joint_trajectory`; electric/pneumatic gripper demos
add `/gripper_controller/follow_joint_trajectory` for `left_finger_joint`.
The right mimic finger has no independent command interface.

These launchers always load `mock_components/GenericSystem`. Connecting RCB
requires a separate hardware/trajectory interface and verified hardware limits;
the demo is not a real-robot launcher. MoveIt plans geometric motion and
time-parameterizes trajectories; it does not validate dynamics or grip force.

## Troubleshooting

- **Package not found:** source both ROS and your workspace's `install/setup.bash`.
- **Missing meshes:** install the matching description package from the same
  release; keep the complete package together.
- **No current state / execution unavailable:** stop other displays/demos and
  wait for `joint_state_broadcaster`, `arm_controller` and, if present,
  `gripper_controller` to become active.
- **No plan:** start from `home`/`ready`, check joint limits, check for collisions
  and choose a reachable TCP target. A target inside the robot should fail.
- **KDL root inertia warning:** KDL ignores root-link inertia for its kinematic
  solve. The original base inertial data remains present.
- **No 3D sensor plugin for octomap updates:** this demo has no depth-sensor
  integration. Planning-scene collision objects still work normally.

Sources: [MoveIt URDF/SRDF concepts](https://moveit.picknik.ai/main/doc/examples/urdf_srdf/urdf_srdf_tutorial.html),
[controller integration](https://moveit.picknik.ai/humble/doc/examples/controller_configuration/controller_configuration_tutorial.html),
[ros2_control mock hardware](https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html).

Models/semantic data and documentation: CC BY 4.0, Source Robotics.
Support code: Apache-2.0. These licenses follow the description repository.
