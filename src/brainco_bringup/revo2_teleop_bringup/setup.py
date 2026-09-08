from glob import glob
from setuptools import setup


package_name = "revo2_teleop_bringup"


setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README_DV1.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/profiles", glob("profiles/*.yaml")),
        ("share/" + package_name + "/tools", glob("tools/*")),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="BrainCo",
    maintainer_email="dev@brainco.cn",
    description="Profile-driven Revo2 hand teleoperation bringup.",
    license="Proprietary",
)
