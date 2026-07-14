import os
from glob import glob

from setuptools import find_packages, setup


package_name = "go2_base_nav"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "behavior_trees"),
            glob("behavior_trees/*.xml"),
        ),
        (
            os.path.join("share", package_name, "rviz"),
            glob("rviz/*.rviz"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yufei",
    maintainer_email="xjtuzhaozhao@gmail.com",
    description="Safe 2D mapping and navigation bringup for a real Unitree GO2.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "planar_odom = go2_base_nav.planar_odom:main",
            "go2_cmd_vel_bridge = go2_base_nav.cmd_vel_bridge:main",
        ],
    },
)
