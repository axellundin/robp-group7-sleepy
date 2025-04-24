from setuptools import find_packages, setup

package_name = 'mapping'

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
    maintainer_email='yassir.fiddi147@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mapping=mapping.ICP_testing:main',
            'mapping_corrected_path=mapping.corrected_path_publisher:main',
            'icp_to_tf_only=mapping.ICP_only_to_tf:main',
            'local_occupancy_map=mapping.local_occupancy_map:main',
            'probabilistic_mapping=mapping.probabilistic_mapping:main',
        ],
    },
)
