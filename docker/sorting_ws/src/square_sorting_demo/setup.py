from glob import glob
import os

from setuptools import find_packages, setup


package_name = "square_sorting_demo"

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
            os.path.join("share", package_name, "worlds"),
            glob("worlds/*.sdf"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ROS 2 developer",
    maintainer_email="developer@example.com",
    description=(
        "Panda shape-recognition and square-sorting simulation based on Chapter 9."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "shape_detector = square_sorting_demo.shape_detector:main",
            "square_sorter = square_sorting_demo.square_sorter:main",
        ],
    },
)
