# Source Robotics robot descriptions

Ready-to-use **URDF and MuJoCo models for PAR6 and PAROL6**, with MSG, SSG48,
flange-only and pneumatic end effectors. Meshes, materials, collision geometry,
joint definitions and physical properties are included in each package.

**Start here:** choose your robot or gripper below, then follow its download and
opening instructions. You do not need to build the models from source.

## Choose your model

| Model | Getting started | Available configurations |
| --- | --- | --- |
| **PAR6** | [PAR6 guide](guides/par6.md) | All six end-effector configurations, URDF and MuJoCo |
| **PAROL6** | [PAROL6 guide](guides/parol6.md) | All six end-effector configurations, URDF and MuJoCo |
| **MSG gripper** | [MSG guide](guides/msg.md) | Solid jaws (default), Soft TPU jaws; mounted on either arm |
| **SSG48 gripper** | [SSG48 guide](guides/ssg48.md) | Solid jaws (default), Fin-ray jaws; mounted on either arm |

![PAROL6 with the six available end-effector configurations](reports/parol6_models.png)

## Download a ready-to-use package

| Robot | MuJoCo | URDF |
| --- | --- | --- |
| PAR6 | [par6_mujoco.zip](https://github.com/Source-Robotics/robot_descriptions/releases/latest/download/par6_mujoco.zip) | [par6_urdf.zip](https://github.com/Source-Robotics/robot_descriptions/releases/latest/download/par6_urdf.zip) |
| PAROL6 | [parol6_mujoco.zip](https://github.com/Source-Robotics/robot_descriptions/releases/latest/download/parol6_mujoco.zip) | [parol6_urdf.zip](https://github.com/Source-Robotics/robot_descriptions/releases/latest/download/parol6_urdf.zip) |

1. Download the ZIP for your robot and simulator from the table.
2. Extract it and keep the whole extracted folder together.
3. Open a `scene_*.xml` file for MuJoCo, or a `par6_*.urdf` / `parol6_*.urdf`
   file for a URDF viewer. The robot guides list the exact files for every option.

The [release page](https://github.com/Source-Robotics/robot_descriptions/releases/latest)
also includes the dimensioned review PDF and checksums. For an online viewer,
upload one extracted model package, including its meshes/textures, rather than
this entire authoring repository. The source checkout contains the same models
under [`mujoco/`](mujoco/) and [`urdf/`](urdf/).

The four release ZIPs contain complete robot assemblies. Dedicated gripper-only
URDF/MuJoCo distributions are not included in this release. The gripper guides
explain the mounted models, isolated MuJoCo grasp fixtures, and the older
standalone exports retained in the gripper hardware repositories.

## End-effector options

| Configuration | File suffix | Notes |
| --- | --- | --- |
| Flange only | `flange` | Physical mounting flange and TCP |
| MSG - Solid jaws | `msg` | Default MSG jaws; includes camera frames |
| MSG - Soft TPU jaws | `msg_tpu_glue` | Yellow honeycomb inserts and black holders |
| SSG48 - Solid jaws | `ssg48` | Default SSG48 jaws |
| SSG48 - Fin-ray jaws | `ssg48_stock` | Alternate SSG48 jaws |
| Pneumatic gripper | `pneumatic` | Two moving jaws, 3.5 mm travel per jaw |

The descriptive jaw names above are the public names. Existing file suffixes are
kept for compatibility. TPU and Fin-ray jaws are currently simulated as rigid
geometry; deformation is not modeled.

## Dimensions, DH tables and physical data

- [Dimensioned review booklet](output/pdf/robot_kinematics_review.pdf): all twelve robot/tool configurations.
- [PAR6 / PAROL6 kinematic comparison](reports/par6_parol6_kinematics.md): axes, DH tables and wrist dimensions.
- [Mass and inertia status](reports/dynamics_sync_validation.md): measured masses, RCB references and remaining estimates.
- [SSG48 collision and grasp checks](reports/ssg48_contact_diagnosis.md): jaw contacts and validation fixtures.

Each model package includes its own `DH.md`, `DYNAMICS.md`, `ACTUATION.md` and
`KINEMATIC_REVIEW.md`. Use those documents when integrating a model with a
controller. Tool COM/inertia, rotor inertia and some actuation parameters remain
estimates; simulation grip checks do not establish hardware grip force.

## Using the shared descriptions from a hardware repository

PAR6, PAROL6, MSG and SSG48 can all reference this repository at
`models/descriptions`. Clicking that submodule on GitHub opens this landing
page at the exact version selected by the hardware repository.

For an existing local checkout of a hardware repository, run:

```bash
git submodule update --init --recursive
```

For a new checkout, use `git clone --recurse-submodules` with that hardware
repository's URL. GitHub's **Download ZIP** does not include submodule contents;
use the model downloads above if you do not use Git. The models are maintained
here and consumers pin an exact commit. See [submodule and release maintenance](RELEASING.md).

## Build or customize

The exported packages work independently. To regenerate them, change jaw colors,
edit branding, or contribute a model, see the [build guide](BUILDING.md),
[model details and authoring notes](MODEL_DETAILS.md), and [changelog](CHANGELOG.md).
The build uses only the files in this repository.

## License

The description packages are provided under [Apache-2.0](LICENSE). Hardware,
firmware and other projects linked from this repository retain their own licenses.
