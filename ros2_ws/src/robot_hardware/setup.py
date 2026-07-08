from setuptools import find_packages, setup

package_name = 'robot_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seventt',
    maintainer_email='chldlstlr6@gmail.com',
    description='Hardware interface nodes: CSI cameras, MCU serial bridges',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'camera_csi_node = robot_hardware.nodes.camera_csi_node:main',
            'mcu_bridge_base_node = robot_hardware.nodes.mcu_bridge_base_node:main',
            'mcu_bridge_arm_node = robot_hardware.nodes.mcu_bridge_arm_node:main',
            'imu_mpu6050_node = robot_hardware.nodes.imu_mpu6050_node:main',
        ],
    },
)
