# Plan to align PAR6 runtime, URDF and MuJoCo dynamics

Based on `rcb_model_comparison.md` and the local runtime source, 2026-10-27.
Updated with the user's measurements and motor-inertia correction, 2026-10-28.
Status 2026-10-28: arm link synchronization and the measured simulation tool
mass partitions are implemented in version 2.8.0. Validation passes at 160
poses per profile. Tool COM/central inertia, runtime tool-mass configuration,
motor rotor calibration and fixed-base alignment remain separate work. See
`dynamics_sync_validation.md` for the implementation and actual results.
The original planning details below are retained as rationale and remaining work.

## Reference and scope

Use the user's measured RCB J1-J5 mass properties as the working arm reference:
`RCB-RUNTIME/RCB-Runtime/robots/PAR6.py`. The actual runtime dynamics builder,
`utility/pinocchio_attach.py::build_pinocchio_dynamics`, consumes these link
mass/COM/inertia values and adds reflected rotor inertia `Jm * G**2`.
Tool data comes from `grippers/*.xml`; tool assembly boundaries and CAD
configuration need confirmation before treating every tool entry as final.
New physical mass targets are recorded in
`source/dynamics/physical_measurements.json`: flange 55.5 g, MSG 380 g,
SSG48 440 g. MSG moving assemblies are approximately 39.3 g each, leaving
301.4 g fixed mass. The user also explicitly confirms SSG48 moving assemblies
are 39.3 g each, leaving 361.4 g fixed mass. These targets supersede the older
configured tool masses.
RCB motor rotor inertias are explicitly NOT a trusted reference: the user
reports they are wrong, and all current models contain estimates. Calibration
is deferred. This does not change the user's preference for RCB arm link
mass, COM and central inertia.

Kinematics already agree within about 0.0035 mm over the audited poses. Keep
the existing joint frames, visual meshes, jaw choices, collisions and TCPs
through the inertial synchronization. The audit's 25 poses are a baseline,
not an exhaustive test over the robot's joint ranges.

Two distinct targets must be tracked:

- Numerical agreement between descriptions using the same physical data.
- Agreement with hardware, including the actual jaw configuration, friction,
  drive response and material compliance. Weighing helps the second target,
  but does not determine COM, inertia or contact behavior by itself.

## 1. Establish one reproducible export path

Add a read-only RCB extraction and model-generation tool under `tools/`, with
a frame/component mapping under `source/dynamics/`. Record source hashes,
units, CAD revision and reference jaw opening. An extracted JSON file should
be a generated snapshot of the reference, rather than an independently edited
second set of arm constants.

Map RCB's DH-link coordinates to each CAD body frame explicitly. Transform
the COM position and rotate the full central inertia tensor into that frame.
Apply parallel-axis terms when combining or splitting physical assemblies;
avoid applying them a second time to an inertia already expressed at its COM.
Validate the mapping over multiple poses and account for existing geometric
rounding. Preserve off-diagonal inertia terms. RCB's arm source uses g mm^2
before its `1e-9` conversion to kg m^2.

URDF should receive mass, inertial origin and full tensor. MJCF can receive
explicit COM and full inertia, allowing the compiler to compute principal
axes. Collision mesh volume must not infer or overwrite these properties.
The jaw builders must consume the same data so a rebuild retains alignment.

Completion: one reproducible source-to-model transformation, with provenance
and a reviewable parameter diff; repeated builds produce the same data.

## 2. Synchronize the five arm links; defer motor rotor calibration

| RCB link | Export body | Reference mass, g |
| --- | --- | ---: |
| J1 | shoulder_link | 1993.14 |
| J2 | upper_arm_link | 1332.24 |
| J3 | forearm_link | 685.11 |
| J4 | wrist_1_link | 579.56 |
| J5 | wrist_2_link | 523.81 |

These are rigid link assemblies, not the mass of each motor alone. Match
their component membership to the measured CAD assemblies. Preserve each
part's assignment to exactly one body.

Correct the older MSG/flange URDF masses and synchronize the rounded native
and SSG48 values to the same reference. Transfer COM and inertia together;
copying the native mass values alone leaves the reported tensor differences.
The native J3 mass is currently 681 g versus 685.11 g in RCB.

Motor rotor values are deferred by the user's 2026-10-28 instruction. Leave
the current consumer-specific estimates unchanged and mark them uncalibrated;
do not copy the incorrect RCB estimates into MuJoCo as measured data.
Compare rigid-body mass matrices with each model's rotor diagonal removed
so these estimates do not obscure arm/tool mass-property alignment.

When correct motor data becomes available, derive the joint-side term from
the motor's rotor inertia and gearbox ratio as `Jm * G**2`, once. Generic URDF
rigid-body inertial elements do not represent joint rotor armature; configure
that term in a supporting simulator/controller. Keep it separate from each
link's spatial inertia.

Completion: all five variants share the same arm link reference after
coordinate conversion. Motor-inertia calibration remains explicitly pending.
An isolated arm-property check can finish before tool measurements arrive.

Also track the fixed base separately. The original comparison intentionally
excluded it from moving-arm dynamics. A follow-up check finds native compiled
base mass about 1116.843 g (inferred from its geometry), MSG/flange URDF base
mass 998.265 g, and SSG48 URDF base mass 553 g. RCB declares a separate
`mbase = 0.410` kg outside the six moving DH links. Confirm the same physical
base assembly is represented before choosing a reference, then supply explicit
base mass/COM/inertia consistently. This affects total robot mass and a future
floating-base model; it does not change the present fixed-base arm torques.

## 3. Resolve end-effector data with targeted measurements and CAD

| Complete tool | RCB configured, g | MuJoCo configured, g | URDF configured, g | Physical target, g |
| --- | ---: | ---: | ---: | ---: |
| Flange | 55.500 | 24.636 | 24.636 | 55.5 |
| MSG | 370.000 | 389.005 | 221.072 | 380 |
| SSG48 | 414.840 | 414.000 | 414.000 | 440 |

The physical targets were supplied by the user on 2026-10-28 and are now
applied to both exported model formats. The configured table above records
the pre-synchronization baseline. MSG has 301.4 g fixed mass
and approximately 39.3 g per moving finger/jaw/slider assembly. The user
explicitly confirms the SSG48 moving assemblies are also 39.3 g each, so its
fixed assembly is `440 - 2 * 39.3 = 361.4 g`.

These simulation totals include all positive-mass tool bodies and fingers,
excluding the fixed robot base. Define the physical mounting boundary before
weighing. Count an adapter/output plate once, whether it belongs to the arm or
the removable tool. Record camera, brackets, rail, motor, fasteners and any
cable segment included in the assembly; exclude an external payload.

Specific findings:

- Flange RCB central inertia is exactly zero, with COM at the tool origin.
  That is a point-mass idealization, not a finite physical flange inertia.
  Obtain the actual COM/tensor from the corresponding CAD assembly before
  using this entry as a physical simulation reference. Do not insert an
  arbitrary small tensor and describe it as a measured match.
- MSG physical total is 380 g, replacing the configured 370 g RCB figure.
  Its measured 39.3 g moving assemblies are heavier than the current MJCF
  and URDF finger-body values. Correct the mass partition as well as the total.
- SSG48 physical total is 440 g, replacing the configured 414.84/414 g
  values. Its existing COM also differs by about 23.37 mm between reference
  and exports. Revisit CAD coordinates, included parts, jaw type and opening;
  correcting the total weight alone cannot determine the right COM.
- The user has already confirmed equal mass for the MSG jaw designs. Keep
  that information unless a new measurement corrects it; their COM and
  inertia can still differ. SSG48's 39.3 g per moving assembly is now confirmed.

For each of MSG / Solid jaws, MSG / Soft TPU jaws, SSG48 / Solid jaws and
SSG48 / Fin-ray jaws, obtain mass properties for the fixed housing and each
complete moving assembly (carriage + jaw + its fasteners). A removed jaw's
mass is useful additional information, but is not the complete finger-body
mass. Document the fully open configuration as the initial aggregate
reference, including actual opening in mm and the CAD coordinate axes.

An aggregate tool mass, COM and tensor cannot uniquely determine the
properties of its three simulation bodies. Use current SolidWorks component
mass properties, informed by the measured component masses and actual
material/mass distribution. The legacy exporter CSV files preserve earlier
component selections and coordinate-system names; use them as provenance,
not as confirmation that their old inertias are correct.

Plain-language meaning: COM is the balance point. Body inertia describes how
the mass is spread and how hard it is to accelerate a rotation about each
axis. Two equal-weight jaws can have different balance points or put more
weight far from an axis, so their inertias differ. This is separate from the
unknown motor rotor inertia. The user does not need to calculate tensors by
hand: the CAD mass-property calculation and frame conversion handle that.
The new scale measurements establish masses; COM/tensors remain labelled
estimates until checked against the corresponding CAD mass distribution.

Export each body's mass, COM, all six central tensor components, units and
coordinate-system transform. Then recombine the three bodies at the chosen
opening and compare their aggregate with the RCB tool entry. Require valid
principal inertias and triangle inequalities; an invalid residual tensor
means the partition/data needs correction, not numerical clamping.

RCB models the tool as one rigid body. A detailed gripper's aggregate inertia
changes as fingers move. Match the chosen reference configuration and report
the opening-dependent differences explicitly. Exact agreement at every
opening would require an opening-dependent runtime tool model; that is a
separate implementation decision after the reference data is settled.

Completion: agreed physical assembly boundaries, jaw-specific moving-body
data and physically valid aggregate tool properties at a recorded opening.
Any proposed correction to RCB tool data receives its own concrete diff.

## 4. Validate numerical agreement before changing hardware behavior

Extend the audit to every tool/jaw profile and poses across the actual RCB
software ranges, including boundary samples and randomized poses.

- Verify each body's mass, COM and full tensor in matching frames.
- Recheck TCP and Jacobians to retain the existing kinematic agreement.
- Compare gravity torque and the six-axis rigid-body arm mass matrix with
  rotor terms removed from both sides while motor calibration is deferred.
  Also report the full configured matrices as diagnostic estimates, without
  requiring them to agree. Hold fingers at the reference opening, with finger velocity
  and acceleration zero. Contacts and drive/friction terms must not enter a
  test intended to compare rigid-body inertia.
- Compare velocity-dependent/inverse-dynamics terms with matching gravity and
  friction conventions, using RCB's dynamics model, not its kinematic helper.
- Sweep jaw openings separately, documenting the expected difference from a
  fixed aggregate runtime tool. Verify left/right coupling and stable motion.
- Re-run clearance, contact/grasp fixtures and local runtime timings. Refine
  contact material parameters only in a separately labelled calibration step.

Proposed initial numerical acceptance targets, to confirm against export
precision: mass within 1 mg, COM within 0.01 mm, central tensor relative error
below 0.01%, maximum gravity discrepancy below 0.001 Nm, and arm mass-matrix
relative error below 0.1% for resolved profiles at the reference opening,
excluding the explicitly deferred rotor terms.
These are software consistency targets, not claims of hardware measurement
accuracy. Report actual maxima; justify any tolerance change.

## 5. Align limits and behavioral parameters as separate work

Use `robots/PAR6.xml` as the source for limits, preserving its distinction
between software travel and physical/cable travel. Classify continuous axes
and actual stops before choosing URDF joint types or MJCF stop ranges.
Keep controller software envelopes separate from mechanical contact stops.
Do not use the unverified per-gripper `qlim_deg` entries as new authority.

Then compare friction, actuator gains, effort limits and drive lag with the
runtime's enabled terms and available measurements. URDF, MuJoCo and RCB do
not all represent friction/transmissions identically. Document approximations
instead of fitting link inertias to compensate for drive or friction errors.
Rigid contacts will continue to approximate Soft TPU and Fin-ray compliance.

Completion: explicit supported/approximated parameter mapping, validated
limits and drive/contact calibration results with their own provenance.

## Suggested first measurements to report

Confirmed measurements are filled below. The existing RCB arm link data can
be used for phase 2. Both gripper mass splits are known; check CAD mass distribution
when available; repeating the confirmed weighing is unnecessary:

| Assembly | Total, g | Left moving assembly, g | Right moving assembly, g | Included parts / opening / scale resolution |
| --- | --- | --- | --- | --- |
| Flange | 55.5 | n/a | n/a | Confirmed by user |
| MSG / Solid jaws | 380 | approx. 39.3 | approx. 39.3 | Fixed assembly 301.4 g |
| MSG / Soft TPU jaws | 380 | approx. 39.3 | approx. 39.3 | MSG jaw designs confirmed equal mass |
| SSG48, current reported assembly | 440 | approx. 39.3 | approx. 39.3 | Fixed assembly 361.4 g; explicitly confirmed |
| Fixed base, if accessible and with assembly scope confirmed | | n/a | n/a | |

A message with the values and included-part list is sufficient. Record scale
resolution and repeated readings rather than adding unsupported decimal
places. COM and inertia should come from the matching CAD assembly/export.
If later arm measurements differ, identify the exact rigid assembly and
included components before replacing its existing reference data.

## Recommended implementation order

1. Build the reference extraction/frame mapping and synchronize arm properties.
2. Use the supplied tool masses and moving-assembly splits, and check CAD
   mass distribution for COM and central inertia.
3. Resolve flange and gripper partitions, then synchronize tool properties.
4. Pass the multi-profile numerical audit and regression checks; rebuild packages.
5. Align limit representation and calibrate drive/contact behavior separately;
   correct motor rotor inertia later when the user supplies suitable data.

The arm synchronization and measured tool-mass steps are complete. Continue
with CAD verification of tool COM and central inertia when that data is available.
