# Standalone gripper validation

Generated from the maintained PAR6 tool subtree, with the root at the physical
`flange` frame. Both arms use the same tool-relative definitions.
`tools/verify_standalone_grippers.py` checks native and URDF exports for both jaw
options: reachable links/joints, required assets, measured masses, unchanged
local inertia, finger travel and FK relative to the original flange.
Native checks also cover the actuator, equality, camera, internal contacts and
finite close/open simulation. Release ZIPs are extracted into temporary folders
and checked without referring back to robot-package assets.

MSG's original mathematical closed endpoint overlaps the opposing jaws by about
0.8 mm. The forced-position sweep verifies this contact matches the source;
the physical close/open cycle instead verifies stable jaw contact, less than
20 microns of opposing-jaw solver compression, and successful reopening.

These are model-preservation and simulation checks, not physical calibration.
