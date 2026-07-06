"""Launch both CSI cameras: top (wide-angle, sensor-id=1) and body (sensor-id=0).

sensor_id matches the hardware inventory: 본체(body)=0, 광각(wide/top)=1.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    common = dict(
        sensor_mode=3, width=1640, height=1232, fps=30, flip_method=0, publish_rate=30.0,
        wbmode=8,   # MUST match the training-capture pipeline (csi_capture.py) or YOLO degrades
    )
    top = Node(
        package="robot_hardware",
        executable="camera_csi_node",
        name="camera_top",
        parameters=[{
            "sensor_id": 1,
            "frame_id": "camera_top",
            "topic": "/camera_top/image_raw",
            **common,
            "flip_method": 2,   # 180-deg rotate: wide cam mounted inverted. perception.yaml
                                # top intrinsics are the matching rotated-frame fisheye solve.
        }],
        output="screen",
    )
    body = Node(
        package="robot_hardware",
        executable="camera_csi_node",
        name="camera_body",
        parameters=[{
            "sensor_id": 0,
            "frame_id": "camera_body",
            "topic": "/camera_body/image_raw",
            **common,
            # body cam training applied fixed WB gains to kill the magenta cast (csi_capture)
            "wb_gains": [1.16, 1.08, 0.82],
        }],
        output="screen",
    )
    return LaunchDescription([top, body])
