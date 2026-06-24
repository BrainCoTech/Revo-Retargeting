from setuptools import find_packages, setup

package_name = 'manus_revo3_retarget'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch',
            ['launch/pipeline_launch.py', 'launch/command_state_viewer.launch.py']),
        ('share/' + package_name + '/config',
            [
                'config/control.yaml',
                'config/thumb_retarget.yaml',
                'config/four_finger_retarget.yaml',
                'config/spread_retarget.yaml',
                'config/retarget_tuning_left_DV1.yaml',
            ]),
        ('share/' + package_name + '/scripts',
            [
                'scripts/run_pipeline_record_mcap.sh',
                'scripts/run_quintic_test_record_mcap.sh',
            ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Example',
    maintainer_email='support@example.com',
    description='Manus to Revo3 retargeting node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command_state_viewer = manus_revo3_retarget.command_state_viewer:main',
            'manus_record = manus_revo3_retarget.manus_record:main',
            'manus_replay = manus_revo3_retarget.manus_replay:main',
            'thumb_cmp_debug = manus_revo3_retarget.thumb_cmp_debug:main',
            'retarget_tuning_panel = manus_revo3_retarget.retarget_tuning_panel:main',
            'joint_state_aligner = manus_revo3_retarget.joint_state_aligner:main',
            'quintic_joint_test = manus_revo3_retarget.quintic_joint_test:main',
        ],
    },
)
