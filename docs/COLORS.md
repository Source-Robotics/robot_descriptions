# Changing model colors

[Back to the repository](../README.md)

There is currently **no global theme file or color-picker GUI**. Badge artwork
and replacement jaws have editable HEX-color configuration. Other parts use
named materials or inline RGBA values in the selected model file. The same
parts can be recolored independently in MuJoCo and URDF.

## Quick example: recolor J1

For an already downloaded MuJoCo package, open the selected robot definition,
such as `par6_msg.xml` or `parol6_msg.xml`. The `scene_*.xml` file includes it.
Find the material named `j1_actuator_black` and change only its `rgba`:

```xml
<material name="j1_actuator_black" rgba="0.10 0.35 0.90 1"
          specular="0.3" shininess="0.25" />
```

Reopen the scene to see blue J1 motor/gearbox visuals. The historical material
name can stay the same even when its color changes.

For URDF, open the selected `.urdf`, find the visual material with the same
name, and change its color:

```xml
<material name="j1_actuator_black">
  <color rgba="0.10 0.35 0.90 1" />
</material>
```

Some URDF material definitions are inline, so change every occurrence for that
part in the selected file. Each configuration is a separate entry file; a
change to `par6_msg` does not automatically recolor `par6_ssg48`. Edit the
corresponding file in the other format if both should match.

RGBA values are **red, green, blue, opacity**, each from 0 to 1. For example,
`#1A59E6` is approximately `0.102 0.349 0.902 1`. Keep opacity at `1` for an
opaque part. Change visual materials/colors only; retain the original collision
geometry, body inertials, joints and actuator settings.

## Which part uses which name?

| Part | MuJoCo / URDF material or visual identifier |
| --- | --- |
| J1 motor + gearbox | `j1_actuator_black`, visual `j1_actuator_visual` |
| J2 motor + gearbox | `j2_actuator_black`, visual `j2_actuator_visual` |
| J3 motor + gearbox | `j3_actuator_black`, visual `j3_actuator_visual` |
| MSG camera lens | `msg_camera_lens_black` |
| MSG motor | `msg_gripper_motor_black` |
| MSG Soft TPU holders/carriages | `msg_jaw_black` |
| MSG Soft TPU honeycomb inserts | `msg_jaw_yellow` |
| SSG48 Solid jaws | `ssg48_jaw_material` |
| MSG Solid jaws / SSG48 Fin-ray jaws | Color on each finger's visual geom/material; inspect `left_finger_link` and `right_finger_link`. |
| Structural parts | Often inline color on the visual geom/material; named visuals include `shoulder_structure_visual`, `upper_arm_structure_visual`, `msg_gripper_structure_visual`. PAROL6 URDF uses `parol6_structure` for its arm. |
| PAR6 logo/cover | Textured visual; edit the badge palette below instead of tinting the whole texture. |

In MuJoCo, a referenced material controls a geom's displayed color. If a visual
has no material, edit that visual geom's `rgba`. Hidden collision geoms often
have `rgba="0 0 0 0"`; leave those alone. URDF colors belong to `<visual>`, not
`<collision>`. STL files contain triangle geometry, not the part's display color.

## Persistent authoring settings

Paths below are relative to the central `robot_descriptions` checkout. The
release ZIPs do not contain its source/generator environment.

| Appearance | File to edit | Setting |
| --- | --- | --- |
| PAR6 side-panel badge | [`source/artwork/par6_side_badge.json`](../source/artwork/par6_side_badge.json) | `colors.background`, `emblem`, `symbol`, `wordmark`, `robot_name`, `tagline`, `divider`; also `panel_color` |
| Badge shapes/text outlines | [`source/artwork/par6_side_badge.svg`](../source/artwork/par6_side_badge.svg) | Editable vector artwork; review the corresponding configuration and cache when shapes change. |
| MSG Soft TPU jaws | [`source/jaws/msg_tpu_glue/jaw.json`](../source/jaws/msg_tpu_glue/jaw.json) | `colors.black` for holders/carriages; `colors.yellow` for honeycomb inserts |
| SSG48 Solid jaws | [`source/jaws/ssg48_pinch/jaw.json`](../source/jaws/ssg48_pinch/jaw.json) | `color` |
| Pneumatic gripper | [`source/pneumatic/model.json`](../source/pneumatic/model.json) | `colors.body`, `colors.jaw` (RGBA) |

For example, the MSG palette starts as:

```json
"colors": {
  "black": "#0F0F11",
  "yellow": "#FFD21A"
}
```

Change `yellow` to `#2080FF` to make the honeycomb inserts blue. Keep the JSON
keys unchanged: they identify parts rather than requiring a particular color.
After installing the [build dependencies](../BUILDING.md), regenerate the
packages. Visual changes also require refreshed dimensioned-review snapshots:

```bash
python tools/build.py
```

If the build stops with the expected stale-snapshot message after a visual
change, refresh the snapshots on a machine with working OpenGL, then finish:

```bash
python tools/build_review_sheets.py --render
python tools/build.py
```

The first build generates the changed arm assets and profiles before the
snapshot check. The refresh then uses those models, and the final build exports
the matching standalone grippers and all eight ZIPs. Changes to badge shape,
source/configuration or texture inputs can invalidate the badge raster cache too.

## What is not centralized yet?

Motor/lens and general arm colors are not in a shared appearance palette.
Direct edits work for viewing, but generated PAROL6/standalone files can be
overwritten by their builders. `tools/split_motor_visuals.py` defines the
initial black materials; it does not overwrite an already installed visual
partition's material on every build. PAROL6 generation reconstructs its arm
visuals from builder defaults, and its current validation expects dark actuator
materials. Supporting arbitrary persistent actuator themes therefore needs a
coordinated generator/validation change; changing the `BLACK` constant alone is
not a complete workflow.

The detailed existing background is retained in each PAR6 package under
`docs/MODEL_DETAILS.md`, sections **Actuator colors**, **Side badge** and
**Jaw options**. This page is the concise starting point for recoloring.
