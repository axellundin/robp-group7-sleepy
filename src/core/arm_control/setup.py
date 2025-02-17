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
            'move_to_pickup = arm_control.move_to_pickup:main'
        ],
    },
)
