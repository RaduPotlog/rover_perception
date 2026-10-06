from setuptools import find_packages, setup

package_name = 'rover_perception_terrain'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/terrain.launch.py']),
        ('share/' + package_name + '/config', ['config/terrain.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='radupotlog',
    maintainer_email='potlogradu@gmail.com',
    description='Ground-plane and slope estimation from the lidar point cloud.',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'terrain_node = rover_perception_terrain.infrastructure.terrain_node:main',
        ],
    },
)
