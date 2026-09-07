from glob import glob
from pathlib import Path
from setuptools import find_packages, setup
setup(name="revohuman_kinematics", version="0.1.0", packages=find_packages(),
      data_files=[("share/ament_index/resource_index/packages", ["resource/revohuman_kinematics"]),
                  ("share/revohuman_kinematics", ["package.xml", "README.md"])] + [
                      ("share/revohuman_kinematics/" + d, [p for p in glob(d + "/*") if Path(p).is_file()])
                      for d in ("config", "urdf", "launch")],
      install_requires=["setuptools"], license="Proprietary",
      maintainer="BrainCo", maintainer_email="dev@brainco.cn",
      description="SDK encoder mapping and URDF FK for RevoHuman",
      entry_points={"console_scripts": ["revohuman_kinematics_node = revohuman_kinematics.node:main"]})
