from setuptools import find_packages, setup

package_name = 'odometry'

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
            'simple_odometry = odometry.hampus_odometry_for_testing_path:main',
            'imu_odometry = odometry.original_odometry:main',
            'tick_accumulator = odometry.tick_accumulator:main',
            'odometry_test = odometry.odometry_test:main',
            'yassirodom = odometry.yassirsodometry:main', 
            'odometry = odometry.odometry:main'
       ],
    },
)
