from glob import glob
from setuptools import find_packages, setup

setup(
    name="revohuman_driver", version="0.1.0", packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/revohuman_driver"]),
        ("share/revohuman_driver", ["package.xml", "README.md"]),
        ("share/revohuman_driver/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="BrainCo", maintainer_email="dev@brainco.cn", license="Proprietary",
    description="RevoHuman raw serial sensor driver",
    entry_points={"console_scripts": [
        "revohuman_node = revohuman_driver.node:main",
    ]},
)
