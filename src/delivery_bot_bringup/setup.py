import os
from glob import glob

from setuptools import setup

package_name = 'delivery_bot_bringup'

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
    description='Top-level bringup, delivery mission manager, and A/B metrics '
                'logger for the social navigation robot.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'delivery_manager = delivery_bot_bringup.delivery_manager:main',
            'metrics_logger = delivery_bot_bringup.metrics_logger:main',
        ],
    },
)
