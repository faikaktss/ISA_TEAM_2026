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
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.dae')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
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
            # Sensor nodes
            'camera_node = otonom_arac.nodes.sensors.camera_node:main',
            'encoder_node = otonom_arac.nodes.sensors.encoder_node:main',
            'lidar_node = otonom_arac.nodes.sensors.lidar_node:main',
            'joystick_node = otonom_arac.nodes.sensors.joystick_node:main',
            # Control nodes
            'control_node = otonom_arac.nodes.control.control_node:main',
            'teensy_node = otonom_arac.nodes.control.teensy_node:main',
            # Perception nodes
            'lane_detection_node = otonom_arac.nodes.perception.lane_detection_node:main',
            'object_detection_node = otonom_arac.nodes.perception.object_detection_node:main',
            # Testing nodes
            'video_player_node = otonom_arac.nodes.testing.video_player_node:main',
            'video_recorder_node = otonom_arac.nodes.testing.video_recorder_node:main',
        ],
    },
)
