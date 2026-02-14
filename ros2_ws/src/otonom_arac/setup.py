import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'otonom_arac'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='faikaktss',
    maintainer_email='148656104+faikaktss@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = otonom_arac.camera_node:main',
            'lane_detection_node = otonom_arac.lane_detection_node:main',
            'lidar_node = otonom_arac.lidar_node:main',
            'object_detection_node = otonom_arac.object_detection_node:main',
            'control_node = otonom_arac.control_node:main',
            'encoder_node = otonom_arac.encoder_node:main',
        ],
    },
)
