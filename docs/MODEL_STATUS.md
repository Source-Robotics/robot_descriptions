# Current model status

[All documentation](README.md)

## Available now

Both arms, standalone MSG/SSG48, every maintained tool/jaw option, visual colors,
branding, collision geometry, dimensioned drawings and portable downloads are
available. Arm link masses/COM/inertias use the captured RCB reference. The
user-reported flange and electric-gripper masses are applied.

## Physical data still to confirm

| Item | Status |
| --- | --- |
| Joint travel limits | Awaiting user-confirmed ranges. The ordinary PAR6 URDF still labels all six arm joints `continuous`; MoveIt applies captured RCB ranges in memory. PAROL6's J6 also retains a `continuous` label with a stored finite range. Apply confirmed limits consistently in a separate model update. |
| Speed and effort limits | Some legacy URDF values are zero placeholders; other drive values are simulation estimates. |
| Motor rotor inertia, friction and drive gains | Calibration deferred; current values are estimates. |
| Tool COM and inertia | Measured total/moving masses do not yet supply verified CAD mass distributions for every tool/jaw option. |
| Jaw gripping points | Confirm a contact/grasp target for each jaw option; existing operational TCPs are retained for compatibility. |
| Pneumatic gripper | Its 195 g total and actuation data remain estimates. |
| Fixed-base mass | Assembly scope remains unresolved; lower priority for fixed-base arm torque calculations. |

The [measured-data file](../source/dynamics/physical_measurements.json) records
what was supplied. [Dynamics reports](../reports/dynamics/README.md) explain
the comparisons. [Jaw-frame notes](../reports/grippers/grasp_frames.md) describe
the proposed measured grasp centers; those proposed frames are not installed.

## Deferred integration work

ROS 2 and MoveIt packages are implemented and tested in Linux CI. User-side
Linux testing and an RCB real-hardware connection are deferred. This folder
cleanup does not change joint limits or physical model properties.
