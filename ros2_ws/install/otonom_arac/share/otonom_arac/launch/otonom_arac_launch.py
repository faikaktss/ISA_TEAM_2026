from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        #Todo:Kamera Node
        Node(
            package='otonom_arac',
            executable='camera_node',
            name='camera_node',
            output='screen',
            emulate_tty=True,
        ),
        
        #Todo:Şerit Algılama Node
        Node(
            package='otonom_arac',
            executable='lane_detection_node',
            name='lane_detection_node',
            output='screen',
            emulate_tty=True,
        ),
        
        #Todo:Lidar Node
        Node(
            package='otonom_arac',
            executable='lidar_node',
            name='lidar_node',
            output='screen',
            emulate_tty=True,
        ),
        
        #Todo:Object Detection Node
        Node(
            package='otonom_arac',
            executable='object_detection_node',
            name='object_detection_node',
            output='screen',
            emulate_tty=True,
        ),
    ])