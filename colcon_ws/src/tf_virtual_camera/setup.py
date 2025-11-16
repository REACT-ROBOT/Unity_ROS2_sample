from setuptools import setup
import os
from glob import glob

package_name = 'tf_virtual_camera'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hijikata',
    maintainer_email='hijikata@todo.todo',
    description='TF subscriber outputting to virtual camera with ShaderMotion encoding',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_camera_node = tf_virtual_camera.tf_camera_node:main',
        ],
    },
)
