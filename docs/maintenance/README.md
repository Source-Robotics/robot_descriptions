# Maintain and rebuild the models

[All documentation](../README.md) · [Folder map](../../REPOSITORY_GUIDE.md)

| Guide | Purpose |
| --- | --- |
| [Build and customize](BUILDING.md) | Install the locked authoring environment, regenerate packages and run checks. |
| [Publish and integrate](RELEASING.md) | Release archives, versioning, hardware guides and software submodule consumers. |
| [Historical authoring notes](MODEL_DETAILS.md) | Background on the original appearance/jaw work; some examples describe older releases. |
| [Folder changes](FOLDER_CHANGES.md) | Where files moved in the repository cleanup. |

Run authoring commands from the repository root. The main entry point remains
`python tools/build.py`; `--models-only` uses the checked CAD/visual assets.
See the [tools index](../../tools/README.md) for focused generators and checks.
