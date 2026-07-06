from setuptools import find_packages, setup
import os

package_name = 'gb_perception'

# Gather data files
data_files = [
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# Install config files
config_dir = os.path.join('config')
if os.path.exists(config_dir):
    for f in os.listdir(config_dir):
        src = os.path.join(config_dir, f)
        if os.path.isfile(src):
            data_files.append(
                (os.path.join('share', package_name, 'config'), [src])
            )

# Install launch files
launch_dir = os.path.join('launch')
if os.path.exists(launch_dir):
    for f in os.listdir(launch_dir):
        src = os.path.join(launch_dir, f)
        if os.path.isfile(src):
            data_files.append(
                (os.path.join('share', package_name, 'launch'), [src])
            )

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='1273375029@qq.com',
    description='Steel Coin (钢镚) perception pipeline: FAST-LIO -> Nav2 point cloud filtering',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'points_filter_node = gb_perception.points_filter_node:main',
            'test_obstacle_cloud_node = gb_perception.test_obstacle_cloud_node:main',
        ],
    },
)
