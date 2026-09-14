from glob import glob
from setuptools import find_packages, setup


package_name = "hand_input_adapters"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="BrainCo",
    maintainer_email="dev@brainco.cn",
    description="Device-specific inputs normalized to HandKinematics.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "humandex_hand_adapter = hand_input_adapters.humandex_adapter_node:main",
            "manus_hand_adapter = hand_input_adapters.manus_adapter_node:main",
            "hex_hand_adapter = hand_input_adapters.hex_adapter_node:main",
        ],
    },
)
