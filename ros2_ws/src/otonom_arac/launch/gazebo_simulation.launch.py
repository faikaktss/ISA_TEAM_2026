#!/usr/bin/env python3
"""
=====================================================
KİŞİ 3 İÇİN TEMPLATE: GAZEBO SİMÜLASYON LAUNCH DOSYASI
=====================================================

GÖREV: Bu dosyayı doldurun
- Gazebo'yu başlatın (test_track.world ile)
- Robot modelini spawn edin (URDF'den)
- ROS2 node'larını başlatın (camera, lidar, control vs.)
- Topic remapping yapın (Gazebo → ROS2 node'lar)
- RViz'i başlatın (opsiyonel, görselleştirme için)

GEREKSİNİMLER:
- Gazebo Classic 11 veya Ignition Gazebo
- ros2_control ile Ackermann controller
- TF transforms (robot_state_publisher)
- Sensor topic'lerinin doğru yönlendirilmesi

TAMAMLANACAK BÖLÜMLER:
1. Gazebo başlatma parametrelerini ayarlayın
2. Robot spawn pozisyonunu belirleyin
3. Node'ları başlatın (otonom_arac_launch.py'den import)
4. Topic remapping yapın
5. RViz config dosyası oluşturun
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # ============================================
    # PAKET YOLU
    # ============================================
    
    pkg_share = FindPackageShare('otonom_arac').find('otonom_arac')
    
    # Dosya yolları
    urdf_file = os.path.join(pkg_share, 'urdf', 'otonom_arac.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'test_track.world')
    rviz_config_file = os.path.join(pkg_share, 'config', 'gazebo_view.rviz')
    
    
    # ============================================
    # LAUNCH ARGÜMANLARI
    # ============================================
    
    # TODO KİŞİ 3: Gerekli launch argümanlarını ekleyin
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Simülasyon zamanı kullan (Gazebo için true)'
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Gazebo world dosyasının yolu'
    )
    
    x_pose_arg = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Robot başlangıç X pozisyonu'
    )
    
    y_pose_arg = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Robot başlangıç Y pozisyonu'
    )
    
    z_pose_arg = DeclareLaunchArgument(
        'z_pose',
        default_value='0.5',
        description='Robot başlangıç Z pozisyonu (zeminden yükseklik)'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='RViz görselleştirme aracını başlat'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Gazebo GUI\'yi göster'
    )
    
    
    # ============================================
    # ROBOT DESCRIPTION (URDF İŞLEME)
    # ============================================
    
    # URDF/XACRO dosyasını işle
    robot_description_content = Command([
        'xacro ', urdf_file
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    
    # ============================================
    # ROBOT STATE PUBLISHER (TF transforms)
    # ============================================
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    
    # ============================================
    # JOINT STATE PUBLISHER (Opsiyonel)
    # ============================================
    
    # TODO KİŞİ 3: Eğer GUI ile joint kontrolü istiyorsanız aktif edin
    # joint_state_publisher_gui_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     output='screen',
    #     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    # )
    
    
    # ============================================
    # GAZEBO BAŞLATMA
    # ============================================
    
    # TODO KİŞİ 3: Gazebo Classic veya Ignition seçin
    
    # Seçenek 1: Gazebo Classic
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', 
             LaunchConfiguration('world'),
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    
    # Seçenek 2: Ignition Gazebo (Gazebo Harmonic için)
    # gazebo_ignition = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('ros_gz_sim'),
    #             'launch',
    #             'gz_sim.launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'gz_args': [LaunchConfiguration('world'), ' -r'],
    #         'on_exit_shutdown': 'true'
    #     }.items()
    # )
    
    
    # ============================================
    # ROBOT SPAWN (Gazebo'ya robot ekleme)
    # ============================================
    
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'otonom_arac',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose'),
            '-z', LaunchConfiguration('z_pose'),
        ],
        output='screen'
    )
    
    
    # ============================================
    # ROS2 NODE'LARI (Gerçek robot kodları)
    # ============================================
    
    # TODO KİŞİ 3: Mevcut node'ları başlatın
    
    # Camera node
    camera_node = Node(
        package='otonom_arac',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        # Topic remapping: Gazebo'dan gelen görüntüyü node'a yönlendir
        remappings=[
            ('/camera/image_raw', '/camera/lane/image_raw'),  # Lane detection için
            # TODO KİŞİ 3: Sign recognition kamerası için de remapping ekleyin
        ]
    )
    
    # Lidar node
    lidar_node = Node(
        package='otonom_arac',
        executable='lidar_node',
        name='lidar_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/lidar/scan', '/scan'),  # Gazebo lidar topic'i
        ]
    )
    
    # Lane detection node
    lane_detection_node = Node(
        package='otonom_arac',
        executable='lane_detection_node',
        name='lane_detection_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Object detection node (YOLO)
    object_detection_node = Node(
        package='otonom_arac',
        executable='object_detection_node',
        name='object_detection_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/camera/image_raw', '/camera/sign/image_raw'),  # Sign recognition için
        ]
    )
    
    # Control node
    control_node = Node(
        package='otonom_arac',
        executable='control_node',
        name='control_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            # TODO KİŞİ 3: Serial portları disable edin (simülasyonda gerekmiyor)
            {'arduino_port': '/dev/null'},
            {'teensy_port': '/dev/null'}
        ],
        remappings=[
            # TODO KİŞİ 3: Control output'unu Gazebo Ackermann controller'a bağlayın
            # Örnek: sag_sol ve ileri_geri topic'lerini /cmd_vel'e dönüştürün
        ]
    )
    
    # TODO KİŞİ 3: Encoder ve Joystick node'larını eklemeyin (simülasyonda gereksiz)
    
    
    # ============================================
    # CMD_VEL BRIDGE (Control → Gazebo)
    # ============================================
    
    # TODO KİŞİ 3: Eğer control_node doğrudan /cmd_vel publish etmiyorsa,
    # custom topic'leri /cmd_vel'e çeviren bir bridge node yazın
    
    # Örnek bridge (pseudo-code):
    # cmd_vel_bridge_node = Node(
    #     package='otonom_arac',
    #     executable='cmd_vel_bridge.py',  # Yeni script gerekecek
    #     name='cmd_vel_bridge',
    #     output='screen',
    #     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    # )
    
    
    # ============================================
    # RVIZ (Görselleştirme - Opsiyonel)
    # ============================================
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    
    # ============================================
    # LAUNCH DESCRIPTION OLUŞTUR
    # ============================================
    
    ld = LaunchDescription()
    
    # Argümanlar
    ld.add_action(use_sim_time_arg)
    ld.add_action(world_arg)
    ld.add_action(x_pose_arg)
    ld.add_action(y_pose_arg)
    ld.add_action(z_pose_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(gui_arg)
    
    # Gazebo
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    
    # Robot
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity_node)
    
    # ROS2 Nodes
    ld.add_action(camera_node)
    ld.add_action(lidar_node)
    ld.add_action(lane_detection_node)
    ld.add_action(object_detection_node)
    ld.add_action(control_node)
    # ld.add_action(cmd_vel_bridge_node)  # TODO: Gerekirse ekleyin
    
    # Visualization
    ld.add_action(rviz_node)
    
    return ld
