from setuptools import find_packages, setup
import os

package_name = 'manus_revo2_retarget'

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
    version='0.0.0',
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
        ]),
        ('share/' + package_name + '/launch', [
            'launch/pipeline_launch.py',
            'launch/real_hand_pipeline_launch.py',
        ]),
        ('share/' + package_name + '/tools', [
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
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manus_revo2_retarget_node = manus_revo2_retarget.retarget_node:main',
            'revo2_teleop_controller = manus_revo2_retarget.teleop_controller_node:main',
            'sim_manus_glove_publisher = manus_revo2_retarget.sim_manus_glove_publisher:main',
            'mujoco_joint_state_viewer = manus_revo2_retarget.mujoco_joint_state_viewer:main',
            'mujoco_manus_overlay_viewer = manus_revo2_retarget.mujoco_manus_overlay_viewer:main',
        ],
    },
)
