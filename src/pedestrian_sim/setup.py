import os
from glob import glob

from setuptools import setup

package_name = 'pedestrian_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AshwinderPalSingh',
    maintainer_email='AshwinderPalSingh@users.noreply.github.com',
    description='Kinematic pedestrian simulator publishing ground-truth detections.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pedestrian_simulator = pedestrian_sim.pedestrian_simulator:main',
        ],
    },
)
