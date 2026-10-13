from setuptools import find_packages, setup

package_name = 'vision_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='haneen',
    maintainer_email='haneen@todo.todo',
    description='Vision package for minesweeper robot',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'camera_node = vision_pkg.camera_node:main',
            'vision_node = vision_pkg.vision_node:main',
            'mine_classifier = vision_pkg.mine_classifier:main',
            'map_node = vision_pkg.map_node:main',
        ],
    },
)