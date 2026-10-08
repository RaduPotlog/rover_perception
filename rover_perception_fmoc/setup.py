from setuptools import find_packages, setup

package_name = 'rover_perception_fmoc'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/fmoc.launch.py']),
        ('share/' + package_name + '/config', ['config/fmoc.yaml']),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='radupotlog',
    maintainer_email='potlogradu@gmail.com',
    description='fmoc person tracking: depth cloud -> ADBSCAN -> tracked person.',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'fmoc_node = rover_perception_fmoc.infrastructure.fmoc_node:main',
        ],
    },
)
