import os
from setuptools import find_packages, setup

package_name = 'gb_nav2'

def _recursive_data_files(data_dir):
    paths = []
    for root, dirs, files in os.walk(data_dir):
        for f in files:
            paths.append(os.path.join(root, f))
    install_dir = os.path.join('share', package_name, data_dir)
    return (install_dir, paths)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        _recursive_data_files('launch'),
        _recursive_data_files('config'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='1273375029@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
