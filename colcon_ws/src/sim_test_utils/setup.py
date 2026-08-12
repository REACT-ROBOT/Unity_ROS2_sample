from setuptools import setup

package_name = 'sim_test_utils'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/examples',
         ['examples/test_basics.py',
          'examples/test_contacts_and_obstacles.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Masaaki Hijikata',
    maintainer_email='hijimasa@gmail.com',
    description='pytest utilities for Unity_ROS2_Robot_Simulator test scenarios',
    license='Apache License 2.0',
    entry_points={
        # pytest がインストール済みパッケージから自動でプラグインを読む。
        # これでテスト側は conftest.py 無しで `sim` フィクスチャを使える。
        'pytest11': [
            'sim_test_utils = sim_test_utils.pytest_plugin',
        ],
    },
)
