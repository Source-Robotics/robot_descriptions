# Repository folder map

[Repository home](README.md) · [Product guides](guides/README.md) · [All documentation](docs/README.md)

## Start with what you want to do

| Task | Open |
| --- | --- |
| Use a robot or standalone gripper | [Product guides](guides/README.md), then one complete runtime package. |
| Change colors or the badge palette | [Color guide](docs/COLORS.md) and [source/appearance/colors.json](source/appearance/colors.json). |
| Compare dimensions or DH tables | [PDF booklet](docs/kinematics/robot_kinematics_review.pdf) or [offline comparison](reports/kinematics/review/index.html). |
| Check measured data and pending work | [Current model status](docs/MODEL_STATUS.md) and [dynamics reports](reports/dynamics/README.md). |
| Rebuild or maintain models | [Build instructions](docs/maintenance/BUILDING.md), [tools index](tools/README.md) and [source index](source/README.md). |
| Publish a release | [Release instructions](docs/maintenance/RELEASING.md). |

## Top-level folders

```text
robot_descriptions/
  guides/       Product-specific getting-started pages
  mujoco/       Four portable MuJoCo model packages
  urdf/         Four portable URDF/ROS description packages
  moveit/       Two optional MoveIt configuration packages
  docs/         Usage guides, images, PDF, maintenance notes and releases
  source/       Editable inputs, original exports and checked caches
  tools/        Build, render, package and validation scripts
  tests/        Regression tests
  reports/      Results grouped by topic
  LICENSES/     Support-code license text
  .github/      Build and release automation
  dist/         Local ZIPs/checksums; ignored by Git
```

Each browsing folder has a README explaining its contents. Detailed results
are under `reports/{appearance,dynamics,grippers,kinematics,packages,history}/`.
Generated Linux integration logs belong under `reports/integration/`.

## Files kept at the root

| File | Purpose |
| --- | --- |
| [README.md](README.md) | Public entry point, products and downloads. |
| [REPOSITORY_GUIDE.md](REPOSITORY_GUIDE.md) | This folder map. |
| [CHANGELOG.md](CHANGELOG.md) | Version history. |
| [ATTRIBUTION.md](ATTRIBUTION.md), [LICENSE](LICENSE) | Model credit and CC BY 4.0 scope. |
| [requirements-build.txt](requirements-build.txt), [requirements-build.lock.txt](requirements-build.lock.txt) | Direct and pinned authoring dependencies. |
| [requirements-review.txt](requirements-review.txt) | Optional PDF-review dependencies. |
| `.gitattributes`, `.gitignore` | Protected source bytes, line endings and ignored local files. |

Maintenance documents moved to [docs/maintenance/](docs/maintenance/README.md).
Versioned release notes are in [docs/releases/](docs/releases/README.md), and
the booklet is in [docs/kinematics/](docs/kinematics/README.md).
See [the move list](docs/maintenance/FOLDER_CHANGES.md) for older paths.

## Runtime packages

| Format | Packages |
| --- | --- |
| MuJoCo | `mujoco/par6`, `mujoco/parol6`, `mujoco/msg`, `mujoco/ssg48` |
| URDF | `urdf/par6_description`, `urdf/parol6_description`, `urdf/msg_description`, `urdf/ssg48_description` |
| MoveIt | `moveit/par6_moveit_config`, `moveit/parol6_moveit_config`; each uses its matching URDF package. |

The arm packages include all six tool configurations. Standalone gripper
packages contain both jaw options. Runtime meshes are repeated where necessary
so each model package can be downloaded or moved independently.

Hardware repositories contain ordinary product-guide folders named
`par6_description`, `parol6_description`, `msg_description` and
`ssg48_description`. Those are guide pages and previews. The actual model
packages with those names live under this repository's `urdf/` folder.

## Inside a MuJoCo package

`mujoco/par6/` and `mujoco/parol6/` each contain the complete arm and six tool
configurations. `mujoco/msg/` and `mujoco/ssg48/` contain only that gripper and
its two jaw options. They have a fixed mounting frame and moving fingers.

| File or folder | Purpose |
| --- | --- |
| `scene.xml` | Default viewer scene: MSG for either arm; Solid jaws for either standalone gripper. |
| `scene_<option>.xml` | Viewer scene for one option; includes the model and adds a floor, lighting and viewer settings. Open this for a quick look. |
| `<product>_<option>.xml` | Native MJCF robot/gripper definition: bodies, joints, inertials, visuals, collisions and actuators. Use this when integrating into another MuJoCo world. |
| `assets/` | Meshes and textures referenced by that package. Keep them with the XML files. |
| `assets/visual/` | Mesh partitions that let motors, gearboxes, lens and structural parts use different materials. |
| `assets/branding/` | Textured side-panel visuals, UV meshes/materials and the baked badge PNG. |
| `assets/jaws/` | Alternate-jaw visual geometry and associated meshes. |
| `assets/contacts/` | Small convex collision pieces for the electric-gripper fingers. These approximate contact surfaces; they are not extra physical links. |
| `assets/pneumatic/` | Pneumatic housing/jaw visuals and convex contacts in the arm packages. |
| `bench/` | Arm-package grasp-test scenes: four electric-jaw profiles, each with a box and cylinder. These are movable test fixtures, not full arm models or the public standalone-gripper packages. |
| `bench/profiles.json` | Grasp-fixture settings, object dimensions and test positions. |
| `bench/README.md` | How the fixtures and grasp checks work. |
| `sim2bot.robot.json` | Optional package/variant index: entry paths, previews, actuator names and TCP names. MuJoCo itself loads XML without this file. |
| `previews/<product>_<option>.png` | A model preview for documentation/catalog viewers. Not a texture. |
| `docs/kinematics_comparison.svg` | PAR6/PAROL6 frame and dimension comparison, included with arm packages. |
| `docs/kinematics_review/` | Six arm/tool review sheets; see below. |

Not every package needs every asset subfolder. Each contains its own required
assets so it can be moved or downloaded independently. Meshes repeated across
packages support that independence; each model only loads its referenced files.

Arm option suffixes are `flange`, `msg`, `msg_tpu_glue`, `ssg48`,
`ssg48_stock` and `pneumatic`. The older suffixes remain for compatibility:
`msg_tpu_glue` means **MSG Soft TPU**, and `ssg48_stock` means **SSG48 Fin-ray**.
Standalone suffixes are `solid` / `soft_tpu` for MSG and `solid` / `fin_ray` for
SSG48.

Detailed notes are grouped in `docs/`, which has its own clickable index.
Only the README, license, package metadata and model/scene entry files stay at
the root. This layout is shared by all eight runtime packages.

## Inside a URDF package

The four folders are `urdf/par6_description/`, `urdf/parol6_description/`,
`urdf/msg_description/` and `urdf/ssg48_description/`.

| File or folder | Purpose |
| --- | --- |
| `<product>_<option>.urdf` | The selected robot/gripper: links, joints, limits, inertials, visual meshes and collision meshes. Open this in a URDF consumer. |
| `meshes/` | All geometry and textures used by the package, grouped by arm/tool and purpose. `visual/`, `branding/`, `jaws/`, `contacts/` and `pneumatic/` serve the same roles as their MuJoCo counterparts, sometimes nested under a tool folder. |
| `package.xml` | ROS package identity, version, license and dependencies. This XML is not a MuJoCo model. |
| `CMakeLists.txt` | Native ament/colcon installation for ROS 2; conditional catkin path for a sourced ROS 1 environment. |
| `launch/display.launch.py` | ROS 2 display with tool/jaw selection, joint sliders and saved RViz configuration. |
| `launch/display.launch` | ROS 1 launch to display the selected configuration in RViz. |
| `launch/gazebo.launch` | Existing ROS 1/Gazebo launch scaffold in arm packages; simulator/controller setup remains the consumer's responsibility. |
| `config/joint_names.yaml` | Arm joint-name configuration. |
| `config/display.rviz` | Saved ROS 2 RViz view with the product's fixed frame, visual model and scale. |
| `scripts/robot_description.py` | Shared model selection and in-memory ROS 2 `package://` resource resolution. |
| `scripts/ros_urdf.py` | ROS 1 helper for portable `package://` paths. It is not a controller. |
| `sim2bot.robot.json`, previews and review drawings | Package index and documentation, as above. |

URDF describes the mechanism and its physical data. It does not by itself
provide a motor controller; gains and simulator-specific behavior belong in
the consuming software. Native ROS 2 installation/display and retained ROS 1
entry points are explained in [the ROS 2 guide](docs/ROS2.md).



## Detailed notes inside a package

Every runtime package has a `docs/README.md` index. The notes cover colors,
camera views where applicable, kinematics, mass/inertia status, actuation,
collision checks and model history. Arm packages include six dimensioned
SVG/PNG sheets plus numeric JSON under `docs/kinematics_review/`.

## Inputs, results and local files

- [source/README.md](source/README.md) explains editable configurations and preserved source/cache data.
- [tools/README.md](tools/README.md) groups the scripts by purpose and explains their outputs.
- [reports/README.md](reports/README.md) separates current check results from historical records.
- `tests/` contains the 238-case portable regression suite. Linux CI additionally
  checks native ROS display and MoveIt planning/mock execution.
- `.git/`, local virtual environments, `__pycache__/` and `dist/` are Git metadata
  or local build files. They are not model inputs.
