import os
from setuptools import setup

package_name = 'obstacle_substitution'

setup(
    name=package_name,
    version='0.1.3',
    packages=[package_name],
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
    maintainer='Omar Elalami',
    maintainer_email='omar@todo.todo',
    description='Converts LiDAR scan data to circle obstacles for the Follow-The-Gap planner',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_substitution_node = obstacle_substitution.obstacle_substitution_node:main',
        ],
    },
)
