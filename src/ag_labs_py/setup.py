import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ag_labs_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['config.yaml']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.yaml')),
        (os.path.join('share', package_name, 'rviz'),
         glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'pynput', 'pyyaml', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='smart-driving@htwk-leipzig.de',
    description='This package contains prepared code for algorithmic geometry from the Smart Driving team',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'naive_path_follow = ag_labs_py.naive_path_follow:main',
            'rectangle_path_follow = ag_labs_py.rectangle_path_follow:main',
            'wasd_teleop_node = ag_labs_py.wasd_teleop_node:main',
            'obstacle_mover = ag_labs_py.obstacle_mover:main',
            'mock_obstacle_sensor = ag_labs_py.mock_obstacle_sensor:main',
            'odom_to_tf = ag_labs_py.odom_to_tf:main',
            'kinematic_path_follower = ag_labs_py.kinematic_path_follower:main',
            'camera_publisher_node = ag_labs_py.camera:main',
            'mock_costmap_publisher = ag_labs_py.mock_costmap_publisher:main',
        ],
    },
)
