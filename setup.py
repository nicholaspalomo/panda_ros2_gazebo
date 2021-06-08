
from glob import glob

from setuptools import setup

package_name = "panda_ros2_gazebo"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="njpalomo",
    author_email="npalomo@student.ethz.ch",
    maintainer="njpalomo",
    maintainer_email="npalomo@student.ethz.ch",
    keywords=["ROS"],
    classifiers=[
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description="Panda demo in ROS2.",
    long_description="""\
Panda demo in ROS2.""",
    license="Apache License, Version 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "panda_pick_and_place = \
                panda_ros2_gazebo.panda_pick_and_place:main",
        ],
    },
)
