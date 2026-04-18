from launch import LaunchDescription
from launch_ros.actions import Node
#Todo: Tüm gerekli düğümleri başlatan bir başlatma dosyası oluştur
def generate_launch_description():
    return LaunchDescription([
        #Todo:Kamera Node — yüksek frekanslı, log dosyasına
        Node(
            package='otonom_arac',
            executable='camera_node',
            name='camera_node',
            output='log',
        ),
        
        #Todo:Şerit Algılama Node — yüksek frekanslı, log dosyasına
        Node(
            package='otonom_arac',
            executable='lane_detection_node',
            name='lane_detection_node',
            output='log',
        ),
        
        #Todo:Lidar Node — yüksek frekanslı, log dosyasına
        Node(
            package='otonom_arac',
            executable='lidar_node',
            name='lidar_node',
            output='log',
        ),
        
        #Todo:Object Detection Node — yüksek frekanslı, log dosyasına
        Node(
            package='otonom_arac',
            executable='object_detection_node',
            name='object_detection_node',
            output='log',
        ),
        
        #Todo:Control Node (Karar Mekanizması) — önemli state logları, ekranda
        Node(
            package='otonom_arac',
            executable='control_node',
            name='control_node',
            output='screen',
            emulate_tty=True,
        ),
        
        #Todo:Encoder Node — yüksek frekanslı, log dosyasına
        Node(
            package='otonom_arac',
            executable='encoder_node',
            name='encoder_node',
            output='log',
        ),
        
        #Todo:Joystick Node — yüksek frekanslı, log dosyasına
        Node(
            package='otonom_arac',
            executable='joystick_node',
            name='joystick_node',
            output='log',
        ),
        
        #Todo:Teensy Node — yüksek frekanslı, log dosyasına
        Node(
            package='otonom_arac',
            executable='teensy_node',
            name='teensy_node',
            output='log',
        ),

        #Todo:GUI Node (Arayüz) — hata logları önemli, ekranda
        Node(
            package='otonom_arac',
            executable='gui_node',
            name='gui_node',
            output='screen',
            emulate_tty=True,
            additional_env={
                'DISPLAY': ':1',
                'QT_X11_NO_MITSHM': '1',
            },
        ),
    ])



