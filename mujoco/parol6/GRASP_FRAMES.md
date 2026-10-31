# Jaw-specific grasp frames: proposed implementation

Keep `tcp` as the operational frame used by existing RCB telemetry, IK and
Haply control. Add a separate frame named `grasp_center` to each jaw profile.
This proposal does not install guessed offsets into the working models.

1. In SolidWorks, identify the midpoint between the opposing intended contact
   patches at a documented opening. Export its translation and orientation
   relative to the gripper housing frame, in metres and radians.
2. Store that mounting transform, contact depth and reference opening in each
   profile configuration under `source/jaws/`. Existing MSG and SSG48 Solid
   configurations are `msg_tpu_glue/jaw.json` and `ssg48_pinch/jaw.json`; give
   the other two profiles equivalent configurations when adding frames.
3. Generate an empty URDF `grasp_center` link with a fixed joint from
   `msg_gripper_link` or `ssg48_gripper_link`, and an MJCF site of that name
   on the same housing. These introduce no mass or collision geometry.
4. Expose the new frame as an explicit planning target in the application.
   Express desired grasp poses in this frame, then convert them to the
   existing TCP target before calling current RCB IK/control code.
5. Check matching URDF/MJCF poses over arm configurations and jaw openings,
   and verify against a real gauge block/contact patch before hardware use.

For symmetric parallel jaws the midpoint is fixed as they open. A contact
patch that moves with opening requires an opening-dependent transform or a
separate frame attached to each finger; a fixed site alone cannot describe it.
The Fin-ray and Soft TPU models are rigid, so load-dependent contact movement
must be calibrated separately if that precision is needed.

SSG48 Solid jaws are about 23.4 mm shorter than Fin-ray jaws. Keeping the old
TCP makes existing motion behavior compatible, but does not make it coincide
with the contact point of both jaw sets. The new frame makes that distinction
explicit without changing measured arm kinematics or inertial data.
