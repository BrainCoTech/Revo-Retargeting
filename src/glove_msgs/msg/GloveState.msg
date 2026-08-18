# GloveState.msg
# One glove's complete hand measurement: identity, side, tracked skeleton and joint angles.
int32 glove_id
string side
int32 skeleton_node_count
SkeletonNode[] skeleton_nodes
int32 joint_angle_count
JointAngle[] joint_angles
geometry_msgs/Quaternion raw_sensor_orientation
int32 raw_sensor_count
geometry_msgs/Pose[] raw_sensor
