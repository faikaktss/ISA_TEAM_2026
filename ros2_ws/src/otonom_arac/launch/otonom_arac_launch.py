from launch import LaunchDescription
from launch_ros.actions import Node
#Todo: Tüm gerekli düğümleri başlatan bir başlatma dosyası oluştur
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
        
        #Todo:Control Node (Karar Mekanizması)
        Node(
            package='otonom_arac',
            executable='control_node',
            name='control_node',
            output='screen',
            emulate_tty=True,
        ),
        
        #Todo:Encoder Node (Hız Hesaplama)
        Node(
            package='otonom_arac',
            executable='encoder_node',
            name='encoder_node',
            output='screen',
            emulate_tty=True,
        ),
        
        #Todo:Joystick Node (Manuel Kontrol)
        Node(
            package='otonom_arac',
            executable='joystick_node',
            name='joystick_node',
            output='screen',
            emulate_tty=True,
        ),
        
        #Todo:Teensy Node (ROS2-Hardware Köprüsü)
        Node(
            package='otonom_arac',
            executable='teensy_node',
            name='teensy_node',
            output='screen',
            emulate_tty=True,
        ),

        #Todo:GUI Node (Arayüz)
        Node(
            package='otonom_arac',
            executable='gui_node',
            name='gui_node',
            output='screen',
            emulate_tty=True,
            additional_env={
                'DISPLAY': ':0',
                'QT_X11_NO_MITSHM': '1',
                'QT_QPA_PLATFORM_PLUGIN_PATH': '/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms',
            },
        ),
    ])



