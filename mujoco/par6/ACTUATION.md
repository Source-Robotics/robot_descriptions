# Position actuation and open-loop hardware

Open-loop stepper control means that commanded step counts are not corrected
using measured joint position. It does not imply infinite mechanical stiffness.
Real steppers have finite torque and load-angle deflection, and can lose steps
or stall. A position servo in a simulator is an approximation of commanded
position tracking, not evidence that the real arm has a position PID loop.

## Current finite simulation settings

The six arm position actuators retain the PAR6 simulation estimates:

| Parameter | Current value |
| --- | --- |
| Position gain Kp | `500` Nm/rad |
| Explicit actuator Kv | `0` Nms/rad |
| Passive joint damping | `25.0` Nms/rad |
| Passive joint friction loss | `1.5` Nm |
| Actuator torque range | `-300 300` Nm |
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
