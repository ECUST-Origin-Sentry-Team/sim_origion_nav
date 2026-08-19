import os
from glob import glob

from setuptools import find_packages
from setuptools import setup


package_name = 'simulated_obstacle_gui'


setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Origin Navigation Team',
    maintainer_email='maintainer@example.com',
    description='Qt coordinate input for a Nav2 circular simulated obstacle.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'simulated_obstacle_gui = simulated_obstacle_gui.main:main',
        ],
    },
)
