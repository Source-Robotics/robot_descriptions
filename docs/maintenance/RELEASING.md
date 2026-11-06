# Publishing a description release

This repository is the source for ten model distributions and two MoveIt
configuration distributions. Model changes and
generation tools belong here. Hardware repositories contain product guides,
previews and release links. Software consumers can pin this repository as a
submodule when they load models directly from a source checkout.

## Prepare and verify

Use the environment in `docs/maintenance/BUILDING.md`. Update the PAR6 URDF `package.xml` and
both PAR6 `sim2bot.robot.json` package versions together. The build propagates
that version to PAROL6, both standalone grippers and both MoveIt packages. Add notes at `docs/releases/vVERSION.md` and update the
changelog. Run:

```bash
python -I tools/build.py
python -I -m pytest tests -q -p no:cacheprovider
python -I tools/prepare_release.py --tag v2.13.0
```

Review the twelve model configurations using the dimensioned review sheets.
Commit the model source, generated packages, documentation and cached PDF.
`dist/` is ignored; release ZIPs are built from the committed package files.

## Publish

After the branch build passes, create an annotated tag for that exact commit:

```bash
git tag -a v2.13.0 -m "Source Robotics robot descriptions v2.13.0"
git push origin main
git push origin v2.13.0
```

The GitHub Actions workflow tests and builds that tag, checks every ZIP's
version against it, and publishes ten model ZIPs, two MoveIt ZIPs, the review PDF, a source
manifest and SHA-256 checksums. Only the tag's release job has repository
write permission. Release publishing waits for the portable build, ROS display
tests and MoveIt planning/mock-execution tests on both Humble and Jazzy.
Existing releases are not overwritten.

## Update hardware-repository guides

Hardware repositories contain guide folders, without model copies or submodules:

| Product | Guide folder |
| --- | --- |
| PAR6 | `par6_description/` |
| PAROL6 | `parol6_description/` |
| MSG | `msg_description/` |
| SSG48 | `ssg48_description/` |
| RAFT | `raft_description/` |

Keep the canonical guide in `guides/` here. After publishing the model release,
export its guide and local previews to the corresponding hardware checkout:

```bash
python tools/export_hardware_guide.py --product par6 --revision CENTRAL_COMMIT --output /path/to/PAR6-Collaborative-Robot-Arm/par6_description
```

Use `parol6`, `msg`, `ssg48` or `raft` for the other products. `--revision` selects the
central commit/tag for documentation and model-file links. The tool exports
only README and preview images; it does not copy model packages. Review and
commit those documentation files in the hardware repository. Model ZIP links
remain direct downloads from the central release.

## Pin models in a software project

Use a submodule when a software application actually loads these model files
from its checkout. From that software repository:

```bash
git submodule add https://github.com/Source-Robotics/robot_descriptions.git external/robot_descriptions
git -C external/robot_descriptions checkout v2.13.0
git add .gitmodules external/robot_descriptions
git commit -m "Pin tested robot description models"
```

The application can select `external/robot_descriptions/mujoco/par6`,
`mujoco/parol6`, `mujoco/msg`, `mujoco/ssg48` or `mujoco/raft`, or the corresponding
`urdf/PRODUCT_description` package. Every path is relative to that submodule's
root. Clone these software consumers with `git clone --recurse-submodules URL`,
or initialize an existing clone with `git submodule update --init --recursive`.

Update deliberately by fetching tags inside the submodule, checking out the
new tested release and committing its pointer in the consuming software
repository. The hardware guide repositories do not need that dependency.
