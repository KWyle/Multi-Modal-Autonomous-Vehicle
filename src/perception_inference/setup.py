from setuptools import setup

package_name = 'perception_inference'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', ['perception_inference/models/policy.onnx']),
        ('share/' + package_name + '/launch', [
            'launch/obstacle_stop.launch.py',
            'launch/ml_infer.launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='wongkyle04@gmail.com',
    description='Perception + inference nodes',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'obstacle_stop_node = perception_inference.obstacle_stop_node:main',
            'infer_node = perception_inference.infer_node:main',
        ],
    },
)
