from setuptools import setup
from glob import glob
import os

package_name = 'follow_the_gap'

launch_files = glob('launch/*.py')
config_files = glob('config/*.yaml')

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# Only install launch/config if they exist and contain files
if launch_files:
    data_files.append((os.path.join('share', package_name, 'launch'), launch_files))

if config_files:
    data_files.append((os.path.join('share', package_name, 'config'), config_files))

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mxck',
    maintainer_email='mxck@todo.todo',
    description='Follow-the-Gap / Autonomy Tester for MXCarkit',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'straight_driver = follow_the_gap.follow_the_gap_node:main',
        ],
    },
)