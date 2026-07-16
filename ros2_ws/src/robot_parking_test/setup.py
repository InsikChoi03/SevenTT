import os
from glob import glob

from setuptools import find_packages, setup

package_name = "robot_parking_test"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "web"), glob("web/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="seventt",
    maintainer_email="chldlstlr6@gmail.com",
    description="Independent arrival-marker reverse parking test nodes",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "parking_controller_node = robot_parking_test.nodes.parking_controller_node:main",
            "parking_web_node = robot_parking_test.nodes.parking_web_node:main",
        ],
    },
)
