from setuptools import find_packages, setup
import os
from glob import glob
# This setup.py file is used to build the ROS 2 package for the rover.
# It specifies the package name, version, and other metadata.
# It also includes data files such as launch files and URDF files.
# The package is named 'rover' and is versioned as '0.0.0'.

package_name = 'rover'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aanimesh',
    maintainer_email='aanimesh@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'avoid_obstacle = rover.obstacle_avoiding:main',
            'gap_follower = rover.gap_follower:main',
        ],
    },
)