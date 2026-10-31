# PAROL6 dynamics inheritance

The user specified that PAROL6 has the same mass properties as PAR6. Each
format therefore inherits PAR6's mass, local COM and full central inertia
in corresponding CAD link frames. Those L1-L5 CAD frames have the same local
part convention; the changed joint axes affect motion, not the meaning of a
local mass property. Tool inertials stay in their shared tool frames after the
explicit L6-to-flange adapter. Native drive/armature estimates are inherited.

The old masses/tensors in the supplied SolidWorks URDF are retained only in
the preserved authoring source. They do not override the preferred PAR6 data.
The native fixed-base estimate is copied explicitly, preventing the new base
mesh volume from silently changing it. Its scope remains unverified.

Shared physical tool totals: flange 0.0555 kg; MSG 0.380 kg; SSG48 0.440 kg.
Each MSG/SSG48 moving finger/jaw/slider is 0.0393 kg. Tool COM/tensors and motor rotor
inertias remain the existing unverified estimates; equal jaw masses do not
establish equal COM or inertia for different shapes.

The pneumatic assembly uses an unverified legacy 0.195 kg total, distributed
by CAD volume with COM/central tensors computed from those solids. It is not
included in the user-measured MSG/SSG48 mass claims. See PNEUMATIC.md.

Public PAROL6 and PAR6 joint axes/origin orientations now match, including
J6. Both forearms are corrected to 177 mm. Their generalized dynamics can
still differ because PAROL6's shorter wrist places each tool 6.6 mm closer
to J6, changing leverage despite identical local mass/COM/tensors. The
lower-arm visual/collision placement is shifted 0.65 mm to align with the
corrected wrist origin; inherited PAR6 local inertials are retained.
The original export's
reversed axes are documented only as a historical source conversion.
Neither actuator calibration nor physical PAROL6 validation is established
by inheriting these parameters. Finite position gains and torque limits
remain existing estimates; see ACTUATION.md.
