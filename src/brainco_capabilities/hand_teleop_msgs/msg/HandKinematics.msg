# One timestamped, palm-local kinematic observation for exactly one hand.
#
# Contract:
# - header.stamp is the source measurement time whenever the source provides it.
# - header.frame_id names a right-handed palm-local frame for this side only.
# - joint_positions_rad aligns one-to-one with joint_names and uses radians.
# - *_tip denotes the glove/model fingertip, with DIP-local offsets applied upstream.
# - finger-pad contact references are separate landmarks, not aliases for *_tip.
# - landmarks_m aligns one-to-one with landmark_names and uses metres.
# - missing measurements are omitted; adapters must not synthesize fake values.

uint8 LEFT=0
uint8 RIGHT=1

std_msgs/Header header
uint8 side
string source

string[] joint_names
float64[] joint_positions_rad

string[] landmark_names
geometry_msgs/Point[] landmarks_m
