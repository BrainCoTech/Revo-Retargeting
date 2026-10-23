from glob import glob
from setuptools import find_packages, setup

package_name = "manus_revo3_vector_retarget"
setup(
    name=package_name, version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/scripts", glob("scripts/*.sh")),
        ("share/" + package_name, ["README.md", "UPSTREAM.md", "requirements.txt"]),
    ],
    install_requires=["setuptools", "numpy>=1.24,<2", "scipy>=1.10,<1.16",
                      "mujoco>=3.3.7,<4", "PyYAML>=5.4,<7"],
    zip_safe=False,
    maintainer="1Jimmy-Chen", maintainer_email="1131359622@qq.com",
    description="MANUS to Revo3 vector-based retargeting, simulation and hardware adapter",
    license="Proprietary",
    entry_points={"console_scripts": ["retarget = retarget_v2.cli:main"]},
)
