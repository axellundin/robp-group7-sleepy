from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'navigation'
core_interfaces_action_path = os.path.join(
    os.path.dirname(__file__),  # Sökvägen till setup.py
    '..',                       # Gå en nivå upp till src/core
    'core_interfaces',
    'action',
    'MoveTo.action'
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
            'geofence_compliance = navigation.geofence_compliance:main',
            'path_planner = navigation.path_planner:main',
            'geofence_publisher = navigation.geofence_publisher:main',
        ],
    },
)
