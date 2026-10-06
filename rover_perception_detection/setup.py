from setuptools import find_packages, setup

package_name = 'rover_perception_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/detection.launch.py']),
        ('share/' + package_name + '/config', ['config/detection.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='radupotlog',
    maintainer_email='potlogradu@gmail.com',
    description='Lightweight ONNX object detection.',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'detection_node = rover_perception_detection.infrastructure.detection_node:main',
        ],
    },
)
