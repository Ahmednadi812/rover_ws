from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'rover_map'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'maps'),
            glob('maps/*')
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*')
        ),
        (
            os.path.join('share', package_name, 'rviz'),
            glob('rviz/*')
        ),
         (
        'share/' + package_name + '/rviz',
        [
            'rviz/simulation.rviz'
        ]
    ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='deyaa',
    maintainer_email='deyaa@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'annotated_map_publisher=rover_map.annotated_map_publisher:main',
            'set_init_pos=rover_map.set_init_pos:main',
        ],
    },
)
