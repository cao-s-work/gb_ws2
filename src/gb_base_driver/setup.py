from setuptools import find_packages, setup

package_name = 'gb_base_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/base_driver.launch.py',
            'launch/real_adapter.launch.py',
        ]),
        ('share/' + package_name + '/config', ['config/base_driver.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='1273375029@qq.com',
    description='钢镚底盘驱动 (Mock)',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gb_base_driver_node = gb_base_driver.base_driver_node:main',
            'real_base_adapter = gb_base_driver.real_base_adapter:main',
            'zenoh_readonly_adapter = gb_base_driver.zenoh_readonly_adapter:main',
        ],
    },
)
