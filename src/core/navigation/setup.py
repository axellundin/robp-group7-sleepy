from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'navigation'
core_interfaces_action_path = os.path.join(
    os.path.dirname(__file__),  
    '..',                       
    'core_interfaces',
    'action',
    'MoveTo.action'
)

core_interfaces_srv_path = os.path.join(
    os.path.dirname(__file__),  
    '..',                      
    'core_interfaces',
    'srv',
    'GridCreator.srv'
)
core_interfaces_srv_Move = os.path.join(
    os.path.dirname(__file__),  
    '..',                      
    'core_interfaces',
    'srv',
    'MoveTo.srv'
)

core_interfaces_srv_MoveToObject = os.path.join(
    os.path.dirname(__file__),  
    '..',                      
    'core_interfaces',
    'srv',
    'MoveToObject.srv'
)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/core_interfaces/action', [core_interfaces_action_path]),
        ('share/core_interfaces/srv', [core_interfaces_srv_path]),
        ('share/core_interfaces/srv', [core_interfaces_srv_Move]),
        ('share/core_interfaces/srv', [core_interfaces_srv_MoveToObject]),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sleepy',
    maintainer_email='axellundin@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_to = navigation.move_to:main',
            'generate_pose = navigation.random_pose_client:main',
            'generate_pose_service = navigation.random_pose_service:main',
            'geofence_compliance = navigation.geofence_compliance:main',
            'path_planner = navigation.path_planner_yassir:main',
            'geofence_publisher = navigation.geofence_publisher:main',
            'grid_map_creator = navigation.grid_map_creator:main',
            'explorer = navigation.explorer:main',
            'pick_up = navigation.pick_up:main',
            'odom_calibration = navigation.odometry_calibration_client:main',
            'Working_explorer = navigation.Working_explorer_3,3min:main',
        ],
    },
)
