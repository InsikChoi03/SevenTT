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
            # GPU downscale 1640x1232 -> 1280x960 (full FOV kept). Cuts per-frame CPU ~40% for 15 Hz.
            # wide fisheye intrinsics in test_field.yaml are scaled to match (top_fx/fy/cx/cy);
            # dist_coeffs + wide_ground.npz are resolution-invariant and unchanged.
            "out_width": 1280,
            "out_height": 960,
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
            # With a fixed body light, keep the near-field classifier input stable by locking
            # exposure/gain instead of letting nvargus 3A chase every cube highlight.
            "exposuretimerange": "8000000 8000000",
            "gainrange": "1 1",
            "aelock": True,
            "awblock": True,
            # GPU downscale 1640x1232 -> 640x480 (cube.pt trains at imgsz 640, so this is its native
            # size). Body detection PIXELS are upscaled x2.5625/x2.5667 back to the 1640 calibration
            # domain in world_model + mission_fsm before the body homography (npz files untouched).
            "out_width": 640,
            "out_height": 480,
        }],
        output="screen",
    )
    return LaunchDescription([top, body])
