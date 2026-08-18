# SkeletonNode.msg
# One node of the tracked hand skeleton (chain of nodes from wrist to fingertips).
int32 node_id
int32 parent_node_id
string joint_type
string chain_type
geometry_msgs/Pose pose
