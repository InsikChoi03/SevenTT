from setuptools import find_packages, setup

package_name = 'robot_control'

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
    description='Control nodes: mecanum base IK, 6-DOF arm controller, go-to-goal, pick sequencer',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'base_controller_node = robot_control.nodes.base_controller_node:main',
            'arm_controller_node = robot_control.nodes.arm_controller_node:main',
            'go_to_goal_node = robot_control.nodes.go_to_goal_node:main',
            'pick_sequencer_node = robot_control.nodes.pick_sequencer_node:main',
        ],
    },
)
