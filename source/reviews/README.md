# Orthographic snapshot inputs

`render_cache/` contains twelve zero-pose native MuJoCo snapshots and their
camera/registration metadata. They are source artwork for the generated
dimensioned sheets, not simulator textures. Every sheet embeds its snapshot,
so exported packages need no reference back to this directory.

Refresh with `python tools/build_review_sheets.py --render` using the locked
MuJoCo 3.3 environment and a working OpenGL backend. The common camera, pose
and drawing layout are defined in that tool. The snapshot silhouette must
remain inside the common camera for every configuration.

Normal headless builds verify canonical geometry/material/scene input hashes
and the cached image checksum. Inertials and actuator elements are excluded
from the visual signature; changing physics alone cannot alter a zero-pose
snapshot. Source inputs are hashed instead of compiled mesh principal axes
to avoid platform-specific floating-point differences. JSON records the
actual orthographic OpenGL frustum and an independent pixel-registration
check using three temporary foreground markers. Those markers are absent
from the saved source images.

Dimensions, DH rows and FK diagnostics are always regenerated from models;
they are never read from a snapshot or manually maintained image annotations.
