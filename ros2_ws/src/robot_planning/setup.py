from setuptools import find_packages, setup

package_name = 'robot_planning'

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
    description='Mission planning / judgment nodes: FSM decision-maker and target selector',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mission_fsm_node = robot_planning.nodes.mission_fsm_node:main',
            'target_selector_node = robot_planning.nodes.target_selector_node:main',
            'explorer_node = robot_planning.nodes.explorer_node:main',
        ],
    },
)
