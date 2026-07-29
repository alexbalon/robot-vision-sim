import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_vision_gazebo'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*.urdf'))),
        (os.path.join('share', package_name, 'worlds'),
            glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Joffre L. León Veas',
    maintainer_email='jleon@example.com',
    description='Proyecto Robot-Vision: navegación autónoma basada en visión artificial (Gazebo + ROS2).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'vision_node = robot_vision_gazebo.vision_node:main',
        ],
    },
)
