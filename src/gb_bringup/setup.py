from setuptools import find_packages, setup
import os

package_name = 'gb_bringup'
share_dir = os.path.join('share', package_name)

src_dir = os.path.dirname(os.path.abspath(__file__))

# Collect launch files
launch_files = []
launch_dir = os.path.join(src_dir, 'launch')
if os.path.isdir(launch_dir):
    for f in os.listdir(launch_dir):
        if f.endswith('.launch.py'):
            launch_files.append(os.path.join('launch', f))

# Collect scripts
script_files = []
script_dir = os.path.join(src_dir, 'scripts')
if os.path.isdir(script_dir):
    for f in os.listdir(script_dir):
        fp = os.path.join(script_dir, f)
        if os.path.isfile(fp) and os.access(fp, os.X_OK):
            script_files.append(os.path.join('scripts', f))

# Collect config files
config_files = []
config_dir = os.path.join(src_dir, 'config')
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
    description='Gangbeng robot bringup: unified launch for Nav2',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [],
    },
)
