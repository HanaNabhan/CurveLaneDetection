from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'track_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Include all files in the launch folder
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        
        # Uncomment the line below to include all YAML config files if needed
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hana Nabhan',
    maintainer_email='hana402947@feng.bu.edu.eg',
    description='Package for track racing navigation and mapping',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add the track_race.py executable to console scripts
            'track_race_node = track_navigation.track_race:main',
            'track_control = track_navigation.control:main',

            
        ],
    },
)
