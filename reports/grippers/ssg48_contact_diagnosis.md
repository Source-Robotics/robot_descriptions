# SSG48 internal contacts and grasp fixture correction

Investigation: 2026-10-31, description version 2.12.1. Applies to both robot
families and both SSG48 jaw options.

## Internal transmission racks

The URDF visual exports include longer toothed transmission racks than the
native MuJoCo exports. Previously the URDF collision decomposition included
those racks, and its convex pieces overlapped the opposing moving assembly
near closure. A 101-point stroke sweep found up to 4.4925 mm penetration for
Solid jaws and 1.5978 mm for Fin-ray jaws. These are collision-hull overlaps,
not measurements of physical parts intersecting in the real gripper.

URDF now uses the same six external carriage/jaw collision pieces per finger
as the native package. Complete rack visuals remain present. The existing
finger mimic/equality constraint represents the internal transmission; the
model does not simulate tooth/gear engagement. All link masses, local COMs,
central inertias, joints, travel, drives, TCPs and visible assets are retained.
The native robot collision geometry is unchanged.

`tools/verify_ssg48_contacts.py` checks 201 travel positions from fully open
to fully closed for both options on both robots, using native models and
the actual URDF collision meshes imported into MuJoCo. All eight combinations
pass. The largest residual is approximately 0.000002 mm at touching solid-jaw
tips, far below the 0.001 mm penetration tolerance.

Housing and slider parts overlap geometrically as expected in a prismatic
guide. Native MuJoCo filters adjacent parent/child collisions; isolated grasp
fixtures explicitly exclude housing/finger pairs. URDF consumers that enable
self-collision must likewise filter each finger against `ssg48_gripper_link`.
The sweep verifies MuJoCo's filtering, not every URDF consumer's configuration.
External jaw/object and housing/object collisions remain enabled.

## The failed box grasp

The former fixture placed the same 60 mm tall, 50 g object under every jaw.
With the shorter SSG48 Solid jaws, both the box and cylinder initially
penetrated the housing collision shape by 13.6998 mm. During closing, the
housing pressed the object against the table. The earlier 31.6 mm box-lift
result therefore came from an invalid initial fixture. No finger/finger or
finger/housing contact was active during that native grasp.

The fixture builder now measures clearance against the actual fixed collision
geometry at the grasp location. Objects retain their 50 g benchmark mass and
are up to 60 mm tall, shortened only when needed to leave 5 mm housing
clearance. SSG48 Solid jaws use a 41.2002 mm object. The other three jaw
profiles retain 60 mm objects. A 0.1 mm initial table gap lets each object
settle before closing and avoids a degenerate exact-face contact at startup.

The corrected Solid-jaw box retains approximately 37.3 mm of the commanded
50 mm lift and releases successfully. All eight grasp cases pass in each
robot's package, for 16 passing cases total. Gripper force, gains, friction,
inertial properties and the 35 mm retained-lift / 25 mm release-drop thresholds
were not changed. Object inertia follows its corrected benchmark dimensions.
Some simulated settling/slip remains; these are regression checks, not a
calibration of hardware grip force, friction, or finger compliance.

## Reproduce

From the authoring repository root:

```bash
python tools/verify_ssg48_contacts.py
python tools/grasp_check.py --all --robot par6
python tools/grasp_check.py --all --robot parol6
python -I -m pytest tests -q -p no:cacheprovider
python -I tools/build.py
```

The full build runs the contact sweeps and all 16 grasp cases before packaging.
Numeric results are in `reports/grippers/ssg48_internal_contacts.json` and
`reports/grippers/grasp_checks.json`. Each exported package includes this document as
`SSG48_CONTACTS.md`.
