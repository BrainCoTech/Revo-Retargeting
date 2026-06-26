from setuptools import find_packages, setup

package_name = 'hex_glove_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jiimmy',
    maintainer_email='1131359622@qq.com',
    description='ROS2 driver bridge for Hexacercle glove UDP data',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hex_glove_udp_node = hex_glove_driver.hex_glove_udp_node:main',
        ],
    },
)
