import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'kerfur_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.py'))),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roo',
    maintainer_email='roo@todo.todo',
    description='Kerfur system bringup: launch files and global config',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
        ],
    },
)
