from setuptools import setup
import os

package_name = 'perception_inference'
model_rel = os.path.join('perception_inference', 'models', 'policy.onnx')
model_entry = []
if os.path.exists(model_rel):
    model_entry = [('share/' + package_name + '/models', [model_rel])]

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/obstacle_stop.launch.py',
            'launch/ml_infer.launch.py'
        ]),
    ] + model_entry,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Perception + inference nodes',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'obstacle_stop_node = perception_inference.obstacle_stop_node:main',
            'infer_node = perception_inference.infer_node:main',
            'ultrasonic_monitor = perception_inference.ultrasonic_monitor:main'
        ],
    },
)
