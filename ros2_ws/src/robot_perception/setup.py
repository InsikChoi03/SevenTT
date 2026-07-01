from setuptools import find_packages, setup

package_name = 'robot_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seventt',
    maintainer_email='chldlstlr6@gmail.com',
    description='Localization + perception nodes: localizer, YOLOv8n detector, SigLIP gate, world model',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'localizer_node = robot_perception.nodes.localizer_node:main',
            'yolo_detector_node = robot_perception.nodes.yolo_detector_node:main',
            'siglip_gate_node = robot_perception.nodes.siglip_gate_node:main',
            'world_model_node = robot_perception.nodes.world_model_node:main',
            'recognition_viz_node = robot_perception.nodes.recognition_viz_node:main',
            # DEPRECATED (kept on disk, deregistered): yolo_world_node, shape_heuristic_node
            # -> replaced by yolo_detector_node (custom YOLOv8n cube.pt; shape class in the
            #    detection label + best body detection republished on /classification/shape).
        ],
    },
)
