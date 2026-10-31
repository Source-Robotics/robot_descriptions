# Pneumatic profile and review validation - 2.12.0

Validated locally on 2026-10-31 with the locked CPython 3.11 environment.

- All 57 description, reference, geometry, FK, contact/motion and portability tests passed in both the authoring checkout and a fresh independent Git clone.
- A full clean-checkout build passed without sibling API/RCB folders or OpenGL snapshot refresh. Generated tracked contents have no logical Git diff. All four release ZIPs reproduce byte for byte and each contains six selectable configurations with SVG/PNG/JSON review sheets and pneumatic documentation/previews.
- The original side-gripping pneumatic assembly is partitioned into housing/air fittings and two moving jaw assemblies. Tests verify exact preservation of all source triangle records and identical native/URDF visual assets. Each rigid assembly uses six refitted convex contact pieces.
- Both jaws use opposing prismatic axes with 3.5 mm travel and equal positive closing coordinates. URDF mimic and native equality coupling agree. Actual mesh checks show approximately 7.1 mm open and 0.1 mm closed clearance; the TCP remains fixed relative to the housing.
- Both arms passed finite open/close cycles without nonfinite states or MuJoCo warnings, with the actuator force inside +/-20 N and symmetric jaw travel. The finite servo approximates valve endpoints; air pressure, compressibility and valve dynamics are not modeled.
- Pneumatic mass is the unverified legacy 195 g value. Its split, COM and central tensors are CAD-volume estimates, not new physical measurements. Existing measured MSG/SSG48 masses are not applied to it.
- Every PAROL6 profile passed 160-pose source/frame/property checks. All 12 tool-specific DH sheets passed 128-pose URDF/native comparisons. Both robots keep the corrected 177 mm forearm and their 57.1 / 50.5 mm flange references. Pneumatic axial TCP reach is 68.9 / 62.3 mm with 45.25 mm lateral offset.
- All 30 model/scene definition files for the ten earlier configurations are byte-identical to the pre-change baseline. New profile generation does not alter those models or their physics.
- All 12 final A4 PDF pages were rendered and visually inspected. Page count, numbering and pneumatic DH dimensions were checked. Both offline comparison selectors were exercised across all 12 diagrams; their default family pairing remains correct. The six-profile PAROL6 preview gallery was refreshed.
- Source provenance hashes, all native/URDF review-data copies, and all package entry/preview paths were verified. Earlier validation reports are archived alongside this one; current model hashes are in model_definition_baseline.json.

## Release archive SHA-256

- `par6_mujoco.zip`: `bf2e4f59298fc295a5fa8a88e95a38ca90d800ce290882fb38dac6023645ec23`
- `par6_urdf.zip`: `78aa43b2de343de92ecfe58fd6207a8c1aa793490899974e5e1fa4632996f9fe`
- `parol6_mujoco.zip`: `481400d6d640e84c3c49e79ad8bcb1451c41863cf94ab158cc5034e445d16a90`
- `parol6_urdf.zip`: `1b0eeffd374cf1f8180ddb271c8f5343fef15848da73097ecf276e882dc478c2`
