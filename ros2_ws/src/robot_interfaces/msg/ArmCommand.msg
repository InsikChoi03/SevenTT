# 6-DOF arm joint angles for Scipia A2T (MG996R x6).
# Angles in degrees, 0~180 range matching PCA9685 servo PWM map.
std_msgs/Header header
uint16 theta1   # base yaw
uint16 theta2   # shoulder
uint16 theta3   # elbow
uint16 theta4   # wrist pitch
uint16 theta5   # wrist roll
uint16 theta6   # gripper
