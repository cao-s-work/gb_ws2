from setuptools import find_packages, setup
import os

package_name = 'gb_lio'
share_dir = os.path.join('share', package_name)

# Collect launch and config files using absolute paths
launch_files = []
config_files = []

src_dir = os.path.dirname(os.path.abspath(__file__))
launch_dir = os.path.join(src_dir, 'launch')
config_dir = os.path.join(src_dir, 'config')

if os.path.isdir(launch_dir):
    for f in os.listdir(launch_dir):
        if f.endswith('.launch.py'):
            launch_files.append(os.path.join('launch', f))

if os.path.isdir(config_dir):
    for f in os.listdir(config_dir):
        if f.endswith('.yaml') or f.endswith('.yml'):
            config_files.append(os.path.join('config', f))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (share_dir, ['package.xml']),
        (os.path.join(share_dir, 'launch'), launch_files),
        (os.path.join(share_dir, 'config'), config_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='1273375029@qq.com',
    description='LiDAR-IMU odometry package for gangbeng robot',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'icp_localization = gb_lio.icp_localization:main',
        ],
    },
)
