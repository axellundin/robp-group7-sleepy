from setuptools import find_packages, setup

package_name = 'arm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'move_arm_hardcoded = arm_control.move_arm_hardcoded:main',
            'arm_joint_tf_pub = arm_control.arm_joint_tf_pub:main', 
            'rrt_pickup = arm_control.move_to_pickup:main',
            'ik_pickup = arm_control.move_to_pickup_old:main',
            'simple_move = arm_control.simple_move:main',
            'pickup_service = arm_control.pickup_service:main',
            'arm_cv_testing = arm_control.arm_cv_testing:main',
            'box_detection = arm_control.test_box_detection:main',
            'box_position_pub = arm_control.box_position_pub:main',
            'place_service = arm_control.place_service:main',
        ],
    },
)
