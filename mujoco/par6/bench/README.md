# Grasp validation scenes

Four jaw profiles, each with a 50 g box and cylinder. The dynamic gripper is
welded to a mocap target for a prescribed lift; its finger joints, contacts, material coefficients, and
inertial data come directly from the corresponding packaged robot model.
Objects are at most 60 mm tall and leave at least 5 mm below the fixed housing.
The shorter SSG48 Solid jaws therefore use approximately 41.2 mm tall objects.
Each object starts 0.1 mm above the table and settles before closing.
Both initial placement and finger self-contact are checked before accepting
a grasp result; force, friction and pass thresholds are unchanged.

From the description root, run `python tools/grasp_check.py --all` for close/lift/hold/
release checks; add `--robot parol6` to check that package's fixtures.
Use `--profile ssg48-solid --shape box --view` for an animated
MuJoCo viewer, or `--preview PATH.png` for three offscreen snapshots.

The object friction and mass are benchmark settings. Passing is a simulation
regression check and does not establish measured gripping force or TPU flex.
