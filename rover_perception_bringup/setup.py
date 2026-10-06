from setuptools import find_packages, setup

package_name = 'rover_perception_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rover_perception.launch.py']),
        ('share/' + package_name + '/config', ['config/apriltag.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='radupotlog',
    maintainer_email='potlogradu@gmail.com',
    description='Launch for the lightweight perception stack.',
    license='Apache License 2.0',
)
