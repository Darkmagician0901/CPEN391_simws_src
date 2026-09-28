from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'milestone1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch and config files so `ros2 launch milestone1 ...` can find them
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='darkmagician0901',
    maintainer_email='darkmagician0901@todo.todo',
    description='CPEN 391 Milestone 1: AEB safety node and PID wall following',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dist_finder = milestone1.dist_finder:main',
            'pid = milestone1.pid:main',
            'safety_node = milestone1.safety_node:main',
        ],
    },
)
