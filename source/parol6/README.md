# Included PAROL6 source

`original/` preserves the user-supplied SolidWorks URDF export, its package
metadata and all seven STL files. `model.json` records their SHA-256 values,
the visual component assignments and the agreed physical-property policy.
The original package metadata, including its license field, is preserved.

Normal generation uses only this repository. Refresh explicitly with
`python tools/build_parol6.py --import-source PATH`, review the source and
component-assignment changes, and run `python tools/build.py`.

Generated axes and origin orientations now match PAR6, including J6; names
are mapped from L1-L6 to the common descriptive names. Original joint types
are retained, with bounds converted into normalized coordinates. The old
export angles are [q1,q2,-q3,-q4,-q5,-q6] relative to the public package.
The original fixed world attachment is omitted because base_link is the fixed
root. L1-L5 are arm bodies; the
L6 end-tool mesh is preserved here as reference and is replaced by whichever
shared PAR6 tool is selected in the generated model.

The original L1-L5 arm mass properties are superseded by PAR6 properties in
the corresponding CAD part frames, per the user's instruction. New packages
carry the repository's Apache-2.0 declaration. This is model-data inheritance,
not an independent PAROL6 weighing or inertia measurement.

The original source's 176.35 mm forearm and rounded RPY entries are preserved
here. Generated models use the corrected 177 mm forearm, a 0.65 mm outward
lower-arm mesh shift, and the PAR6 reference angles. PAROL6's normalized
wrist-to-flange transform is +50.5 mm on Z with no rotation; PAR6 uses +57.1 mm.
The shorter PAROL6 wrist preserves the same housing-to-tool clearance.
`model.json` records these corrections; they do not alter the original export.

See the generated `docs/KINEMATICS_COMPARISON.md` and diagram in each package for
the standard DH table, coordinate conversion and validation results.
