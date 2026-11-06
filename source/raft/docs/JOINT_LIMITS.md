# Joint ranges

Angles in the model's convention: 0 is the standby pose (PAR6's standby pose), positive about each joint's z axis.
The URDF `<limit>` and the MuJoCo `range` carry the same numbers.

{{LIMITS_TABLE}}

## Where the numbers come from

- **Measured stops.** Every joint has one mechanical stop. Its angle was measured on the first arm (2026-11-02),
  and RAFT's firmware calibrates against it (`kStopDeg` in
  [`firmware/src/main/arm_design.h`](https://github.com/Source-Robotics/RAFT-teleoperation-arm/blob/main/firmware/src/main/arm_design.h)). These are the most trustworthy
  numbers in the table.
- **CAD sweep.** The other end of a range comes from the CAD meshes: one joint turned, the others at 0, until its
  moving parts come within 0.5 mm of the parts it turns against. Where a sweep can be compared with a measured stop
  it ends 0.3 to 2.2 deg early (J2, J3, J5, J6) or 4.6 deg early (J4), so these ends are slightly conservative.
  They hold with the other joints at 0; some wrist poses reach further or less far.
- **Forearm lying limp.** J3's lower end was read on the first arm with the arm limp and the forearm lying folded
  on the upper arm (2026-11-06). The CAD parts meet at about -77 deg; the printed parts settle 3 deg further.
- **Trigger.** 0 is the trigger flush against the handle, as in the CAD; turning it positive would push it into
  the handle. It opens toward negative angles, to its stop 39.63 deg from flush.
- **Not measured.** The CAD has no J1 stop. J1's upper end mirrors its measured stop until the real end is
  measured.

The ranges of one arm can be measured with the firmware's `calib_range` command (move every joint gently to both
ends). Report measured ranges in the [RAFT repository's issues](https://github.com/Source-Robotics/RAFT-teleoperation-arm/issues).

## Keyframes

{{KEYS_TABLE}}

`home` is the standby pose. `parked` is RAFT's firmware rest pose: the arm lying back the way it rests limp.
