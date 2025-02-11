from setuptools import find_packages
from setuptools import setup

setup(
    name='core_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('core_interfaces', 'core_interfaces.*')),
)
