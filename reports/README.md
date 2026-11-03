# Reports and validation

[Current model status](../docs/MODEL_STATUS.md) · [Repository map](../REPOSITORY_GUIDE.md)

Reports record what generators and checks found. Edit the corresponding files
under [source/](../source/README.md) to change model inputs, then rebuild.

| Topic | Contents |
| --- | --- |
| [Mass properties and actuation](dynamics/README.md) | Measured/reference comparisons, synchronization results and remaining estimates. |
| [Kinematics and dimensions](kinematics/README.md) | Frame conversions, dimensions, DH comparisons and PAROL6 build checks. |
| [Grippers and contacts](grippers/README.md) | Jaw geometry, coupling, collision coverage, grasp tests and contact benchmarks. |
| [Appearance and rendering](appearance/README.md) | Motor partitions, side-badge geometry and rendering performance records. |
| [Exported packages](packages/README.md) | ZIP extraction, asset portability and version validation results. |
| [Historical validation records](history/README.md) | Earlier milestones retained as evidence. Read their recorded version before treating a result as current. |
| [ROS integration runs](integration/README.md) | Generated Linux ROS/MoveIt logs and screenshots, normally downloaded from CI artifacts. These are not model inputs. |

Human-readable `.md` notes interpret the structured `.json` records.
Rendering timings are local measurements, not hardware specifications.
ROS/MoveIt CI artifacts are stored under `integration/` when downloaded locally.
