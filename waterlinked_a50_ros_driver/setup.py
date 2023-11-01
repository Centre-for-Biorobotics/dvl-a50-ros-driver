import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'waterlinked_a50_ros_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Roza Gkliva',
    maintainer_email='roza.gkliva@ttu.ee',
    description='The waterlinked_a50_ros_driver package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dvl_tcp_publisher = waterlinked_a50_ros_driver.tcp_publisher:main',
            'dvl_serial_publisher = waterlinked_a50_ros_driver.serial_publisher:main',
        ],
    },
)
