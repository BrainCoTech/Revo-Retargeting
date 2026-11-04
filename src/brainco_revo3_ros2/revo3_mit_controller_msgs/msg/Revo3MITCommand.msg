std_msgs/Header header

# Empty joint_names means arrays follow the controller's configured joint order.
# Non-empty joint_names allows full or partial named commands.
string[] joint_names

# ROS units: position rad, velocity rad/s, effort mA torque feedforward.
# Arrays can be empty to keep/default that field. Non-empty arrays must match
# joint_names length, or the controller joint count when joint_names is empty.
float64[] position
float64[] velocity
float64[] effort
float64[] kp
float64[] kd