# Importing the supplied PAROL6 export

The original URDF names joints and moving links L1 through L6. They are
renamed in order to base_joint/shoulder_link,
shoulder_joint/upper_arm_link, elbow_joint/forearm_link,
wrist_1_joint/wrist_1_link, wrist_2_joint/wrist_2_link and
wrist_3_joint/wrist_3_link. The old fixed world-to-base joint is omitted;
base_link is the fixed package root. Public axes and origin orientations now
match PAR6, including J6. The forearm is corrected to 177 mm; the lower-arm
mesh origin moves -0.65 mm on local Z and J5 moves to -177 mm. The wrist's
flange reach is 50.5 mm, matching PAR6 spacing against the shorter housing.
Source joint types are retained and bounds converted.
New PAR6 and PAROL6 package angles are identical. When migrating trajectories
from the old 2.9.0 raw-export convention, negate joints 3 through 6 once.
The original source URDF remains unchanged for reference.

The original L6 end-tool geometry is replaced by the selected shared tool.
The physical flange adapter and every flange-relative tool TCP are explicit.
Relative to 2.10.0, recompute FK/IK targets using the corrected arm and flange
transforms: the tool is 6.6 mm closer to J6 and the old 176.35 mm forearm is
now 177 mm. This is a model geometry correction, not a joint-angle sign change.
Imported source masses/tensors are superseded by the requested PAR6 values.
ROS launch files convert portable relative mesh paths to package:// URIs
in memory; the portable URDF files on disk are not rewritten.
