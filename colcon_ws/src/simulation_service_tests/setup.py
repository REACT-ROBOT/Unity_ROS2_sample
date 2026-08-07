import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'simulation_service_tests'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Masaaki Hijikata',
    maintainer_email='hijimasa@gmail.com',
    description='Service conformance tests for Unity_ROS2_Robot_Simulator',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service_conformance = simulation_service_tests.run_conformance:main',
        ],
    },
)
