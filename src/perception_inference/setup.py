from setuptools import setup
from glob import glob
import os

package_name = 'perception_inference'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/models', glob('models/*')),  # installs yolo12n.pt
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='YOLO inference node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'infer_node = perception_inference.infer_node:main',
            'ultrasonic_monitor = perception_inference.ultrasonic_monitor:main',
            'obstacle_stop_node = perception_inference.obstacle_stop_node:main',
        ],
    },
)
