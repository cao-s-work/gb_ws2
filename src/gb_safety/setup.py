from setuptools import setup
from glob import glob
import os

package_name = 'gb_safety'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='nvidia@jetson.local',
    description='Safety gate for Gangbeng robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_node = gb_safety.safety_node:main',
        ],
    },
)
