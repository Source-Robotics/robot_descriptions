# PAROL6 DH and joint convention

The generated package uses PAR6 axes and joint-origin orientations, including
J6. Public joint coordinates and the 177 mm forearm match PAR6 directly.
Standard-DH d4 is -177.00 mm on both. Only flange-ending d6 differs:
PAROL6 -50.5 mm, PAR6 -57.1 mm. See the
[complete comparison](KINEMATICS_COMPARISON.md) for the numeric table,
historical-export conversion and validation. The wrist-to-flange offset is
+50.5 mm on local Z with identity rotation. See `ACTUATION.md` for controller
behavior; open-loop hardware does not imply infinite simulated stiffness.
