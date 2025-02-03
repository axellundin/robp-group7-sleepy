from setuptools import find_packages, setup

package_name = 'lidar_processing'

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
    maintainer_email='qinyongze2002@gmail.com',
    description='visualize and process lidar data',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_processing = lidar_processing.lidar_processing:main'
        ],
    },
)
