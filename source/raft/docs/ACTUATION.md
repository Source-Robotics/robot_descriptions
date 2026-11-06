# Actuation

[Documentation index](README.md)

RAFT's seven Feetech HLS3606 servos run in current (torque) mode: the firmware computes gravity, friction and
force-feedback torques itself and sends a current to each servo. The MuJoCo model works the same way.

{{ACTUATOR_TABLE}}

- **Actuators** are `motor` actuators: `ctrl` is the joint output torque in Nm, limited to the servo's stall torque
  (6 kg·cm at 6 V). A simulation sends what the firmware sends; there are no position servos to tune.
- **The URDF** carries the same limits as `effort` (Nm) and `velocity` = {{SPEED}} rad/s, a conservative value
  below the servo's no-load speed.

## Joint dynamics (estimates)

| Value | Model | What it is |
|---|---:|---|
| armature | {{ARMATURE}} kg·m² | the motor rotor reflected through the 1:205 gear (about 0.7 g·cm² × 205²) |
| frictionloss (URDF `friction`) | {{FRICTION}} Nm | gearbox Coulomb friction at the output |
| damping | {{DAMPING}} Nm·s/rad | viscous friction at the output |

These are estimates, not measurements. On the real arm the gears turn so easily from the joint side that the
firmware adds damping on the light wrist joints in gravity mode.

## J2 counterbalance

The MuJoCo model has the optional J2 counterbalance as a tendon, `j2_spring`, between the sites `j2_spring_anchor`
(13 mm above the J2 axis, on `shoulder_link`) and `j2_spring_pin` (on `upper_arm_link`). Its stiffness is 0, so the
spring is not fitted. Set it to the spring's rate in N/m to simulate one; the cable geometry gives a J2 torque of
-k · 13 mm · 20 mm · sin(q2). The mount is described in the RAFT repository's [J2 spring guide](https://github.com/Source-Robotics/RAFT-teleoperation-arm/blob/main/firmware/docs/spring_mount.md).

## Not modelled

Servo current limits and derating with temperature, gear backlash, gear efficiency, and cable forces.
