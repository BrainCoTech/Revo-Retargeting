from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'revo2_hand_retarget'

# Collect all files under brainco_hand so they are installed as package data.
brainco_data_files = []
brainco_base = os.path.join(package_name, 'brainco_hand')
for root, dirs, files in os.walk(brainco_base):
    for f in files:
        # Path relative to the package root
        rel_path = os.path.relpath(os.path.join(root, f), package_name)
        brainco_data_files.append(rel_path)

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: brainco_data_files,
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/retarget.yaml',
            'config/teleop_controller.yaml',
            *glob('config/flexion_*.yaml'),
        ]),
        ('share/' + package_name + '/launch', [
            'launch/pipeline_launch.py',
            'launch/real_hand_pipeline_launch.py',
        ]),
        ('share/' + package_name + '/tools', [
            'tools/analyze_revo2_jitter_bag.py',
            'tools/revo2_retarget_plot.py',
        ]),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'pyyaml',
        'mujoco>=3.0',
    ],
    zip_safe=False,
    maintainer='jackhance',
    maintainer_email='jackhanceli@outlook.com',
    description='Source-neutral hand kinematics retargeting for the Revo2 hand',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'calibrate_dv1_fingers = revo2_hand_retarget.calibrate_dv1_fingers:main',
            'revo2_hand_retarget_node = revo2_hand_retarget.retarget_node:main',
            'revo2_teleop_controller = revo2_hand_retarget.teleop_controller_node:main',
            'mujoco_joint_state_viewer = revo2_hand_retarget.mujoco_joint_state_viewer:main',
        ],
    },
)
