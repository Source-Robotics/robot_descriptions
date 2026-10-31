# Publishing a description release

This repository is the source for all four distributions. Model changes and
generation tools belong here; consuming robot repositories pin a commit of
this repository as a submodule.

## Prepare and verify

Use the environment in `BUILDING.md`. Update the PAR6 URDF `package.xml` and
both PAR6 `sim2bot.robot.json` package versions together. The build propagates
that version to PAROL6. Add notes at `releases/vVERSION.md` and update the
changelog. Run:

```bash
python -I tools/build.py
python -I -m pytest tests -q -p no:cacheprovider
python -I tools/prepare_release.py --tag v2.12.1
```

Review the twelve model configurations using the dimensioned review sheets.
Commit the model source, generated packages, documentation and cached PDF.
`dist/` is ignored; release ZIPs are built from the committed package files.

## Publish

After the branch build passes, create an annotated tag for that exact commit:

```bash
git tag -a v2.12.1 -m "Source Robotics robot descriptions v2.12.1"
git push origin main
git push origin v2.12.1
```

The GitHub Actions workflow tests and builds that tag, checks every ZIP's
version against it, and publishes four model ZIPs, the review PDF, a source
manifest and SHA-256 checksums. Only the tag's release job has repository
write permission. Existing releases are not overwritten.

## Add the source to a robot repository

From the consuming robot repository:

```bash
git submodule add https://github.com/Source-Robotics/robot_descriptions.git models/descriptions
git -C models/descriptions checkout v2.12.1
git add .gitmodules models/descriptions
git commit -m "Add tested robot descriptions submodule"
```

PAR6 uses `models/descriptions/mujoco/par6` or
`models/descriptions/urdf/par6_description`; PAROL6 uses the corresponding
`parol6` paths. Every tool option is included. Clone consumers with
`git clone --recurse-submodules URL`, or initialize an existing clone with
`git submodule update --init --recursive`.

Update a submodule deliberately by fetching tags inside it, checking out the
new release, and committing the new pointer in the consuming repository.
The parent repository records an exact commit, not a moving branch.
