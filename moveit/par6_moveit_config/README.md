# PAR6 MoveIt 2

Motion planning and simulated trajectory execution for all six PAR6
tool/jaw configurations. Requires the matching `par6_description` package
from the same release. No meshes or physical model data are duplicated here.

Follow [installation and the RViz walkthrough](docs/MOVEIT2.md), then launch:

```bash
ros2 launch par6_moveit_config demo.launch.py end_effector:=msg jaws:=soft_tpu
```

Choose the `arm` group and `tcp` tip in RViz; select **Plan**, then **Execute**.
Execution uses `mock_components/GenericSystem`, which follows joint commands
ideally. It tests planning/control integration without simulating loads,
contact forces, motor dynamics or the real RCB controller.

Tool options: `flange`, `msg`, `ssg48`, `pneumatic`. Electric jaw options:
MSG `solid` / `soft_tpu`; SSG48 `solid` / `fin_ray`. Omit `jaws` for flange
and pneumatic. `rviz:=false` runs the same planning/execution stack headlessly.

## Files

- `config/profiles/*.srdf`: groups, TCP, named poses, collision exemptions.
- `config/profiles/*.json`: model selection, initial positions and joint ranges.
- `config/joint_limits.yaml`: editable demo speeds, accelerations and effort placeholders.
- `config/kinematics.yaml`: TCP inverse-kinematics solver.
- `config/ompl*.yaml`: Humble/Jazzy planning pipelines.
- `config/planning_parameters.yaml`: execution and planning-scene settings.
- `config/moveit.rviz`: saved MotionPlanning view.
- `launch/demo.launch.py`: selection, planning, mock controllers and RViz.
- `scripts/model_config.py`: in-memory planning/controller adapter.

Controller settings are assembled for the selected tool, so flange-only has
no gripper controller and mimic fingers are never independently commanded.
The description package's saved URDFs/geometry stay unchanged. See the guide
for joint-limit sources and the MSG closed-preset stand-off.

Model/semantic data and documentation: **CC BY 4.0**, copyright 2026 Source
Robotics; [source](https://github.com/Source-Robotics/robot_descriptions).
Launch/support code and CMake: **Apache-2.0**, see [code license](docs/LICENSE-CODE.txt).
