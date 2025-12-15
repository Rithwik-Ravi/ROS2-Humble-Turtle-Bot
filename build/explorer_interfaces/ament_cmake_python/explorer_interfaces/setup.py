from setuptools import find_packages
from setuptools import setup

setup(
    name='explorer_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('explorer_interfaces', 'explorer_interfaces.*')),
)
