from setuptools import find_packages, setup
import os

package_name = 'gb_web'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/www', [
            'www/index.html',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/gb_web.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='1273375029@qq.com',
    description='钢镚机器人 Web 控制端 ROS2 节点',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'gb_web_node = gb_web.gb_web_node:main',
        ],
    },
)
