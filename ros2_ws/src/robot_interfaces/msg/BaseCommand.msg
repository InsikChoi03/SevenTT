# Mecanum base velocity command in base_link frame.
# Coordinate convention: x forward, y left, z up (ROS REP-103).
std_msgs/Header header
float32 vx       # m/s, forward
float32 vy       # m/s, left
float32 omega    # rad/s, yaw (CCW positive)
