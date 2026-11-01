import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'tb3_explorer'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/tb3_explorer']),
        ('share/tb3_explorer', ['package.xml']),
        (os.path.join('share', 'tb3_explorer','launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tunz',
    maintainer_email='243219810+Tunz10@users.noreply.github.com',
    description='Autonomous frontier-based exploration and mapping for TurtleBot3 using SLAM Toolbox and Nav2.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'explorer = tb3_explorer.explorer:main',
        ],
    },
)
