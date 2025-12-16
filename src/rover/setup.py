from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rover'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        
        # URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        
        # World files
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'worlds', 'my_world'), glob('worlds/my_world/*')),
        
        # --- FIX: This line installs the nav2_params.yaml file ---
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        
        # Map files
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
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