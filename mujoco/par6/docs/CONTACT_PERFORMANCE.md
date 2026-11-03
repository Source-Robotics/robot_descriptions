# Local contact-refinement timing check

Five interleaved rounds, 1280x720 offscreen rendering. Values are median milliseconds.

| Tool | Contacts | Physics step | Full clearance query | Offscreen frame |
| --- | --- | ---: | ---: | ---: |
| MSG | single_hull | 0.031 | 0.264 | 6.536 |
| MSG | convex_pieces | 0.031 | 0.566 | 7.223 |
| SSG48 | single_hull | 0.023 | 0.169 | 5.789 |
| SSG48 | convex_pieces | 0.023 | 0.472 | 5.280 |

Convex contact pieces increase active collision geoms from 10 to 20 and custom
clearance pairs from 36 to 146. A full clearance query costs about 0.30 ms more
for either tool. Physics step timing is essentially unchanged in this fixture.
Frame differences vary in direction between the two tools; rendering includes
scene update and RGB readback, not GUI presentation or Haply I/O. These values
do not measure end-to-end teleoperation latency or guarantee a tick deadline.

See `reports/grippers/gripper_contact_performance.json` in the source description
repository for every sample, configuration and timing range.
