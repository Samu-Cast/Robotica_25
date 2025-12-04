from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'charlie'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #Include launch files
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Samuele Castellani, Samuele Verna, Giammarco Ubaldi',
    maintainer_email='your.email@example.com',
    description='Charlie - Emergency Rescue Robot with Behavior Tree',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'behavior_tree_node = behavior_tree.behavior_tree_node:main',
            'perception_node = perception.perception_node:main',
            'navigation_node = navigation.navigation_node:main',
        ],
    },
)
