#!/usr/bin/env python3
"""
Video Player Node - Teknofest Video Test Sistemi
Video dosyasından frame okuyup ROS2 /camera/image_raw topic'ine publish eder

KULLANIM:
ros2 run otonom_arac video_player_node --ros-args -p video_path:=/path/to/video.mp4
"""
#Todo: Bu kısım videodan gelen veriyi okur ve bir kanala atar bu diğer düğümler ise bu kanaldan verileri alır ve ona göre okur

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os


class VideoPlayerNode(Node):
    def __init__(self):
        super().__init__('video_player_node')
        
        # TODO: Parametreleri tanımla
        self.declare_parameter('video_path', '/Users/faikaktss/teknofest_videos/parkur_2025.mp4')
        self.declare_parameter('fps', 30.0)  # Saniyede kaç frame
        self.declare_parameter('loop', True)  # Video bitince başa sar
        
        # TODO: Parametreleri al
        self.video_path = self.get_parameter('video_path').value
        self.target_fps = self.get_parameter('fps').value
        self.loop_video = self.get_parameter('loop').value
        
        # TODO: Publisher oluştur - gerçek kamera ile aynı topic
        self.image_publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # TODO: OpenCV-ROS dönüşümü için bridge
        self.bridge = CvBridge()
        
        # TODO: Video durumu
        self.video_capture = None
        self.current_frame = 0
        self.total_frames = 0
        
        # TODO: Video dosyasını aç
        if not os.path.exists(self.video_path):
            self.get_logger().error(f' Video bulunamadı: {self.video_path}')
            self.get_logger().info('Örnek: ros2 run otonom_arac video_player_node --ros-args -p video_path:=/path/to/video.mp4')
            return
        
        self.video_capture = cv2.VideoCapture(self.video_path)
        
        if not self.video_capture.isOpened():
            self.get_logger().error(' Video açılamadı')
            return
        
        # TODO: Video bilgilerini al
        self.total_frames = int(self.video_capture.get(cv2.CAP_PROP_FRAME_COUNT))
        #TODO: Video FPS'sini al, eğer alınamazsa target_fps'yi kullan
        self.video_fps = self.video_capture.get(cv2.CAP_PROP_FPS)
        width = int(self.video_capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        # TODO: Timer oluştur - FPS'e göre periyot hesapla
        timer_period = 1.0 / self.target_fps  # Saniyede target_fps kez çalışacak
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # TODO: Başlangıç mesajları
        self.get_logger().info('═' * 70)
        self.get_logger().info('VIDEO PLAYER NODE BAŞLATILDI')
        self.get_logger().info('═' * 70)
        self.get_logger().info(f' Video: {os.path.basename(self.video_path)}')
        self.get_logger().info(f' Çözünürlük: {width}x{height}')
        self.get_logger().info(f' Toplam Frame: {self.total_frames}')
        self.get_logger().info(f' Video FPS: {self.video_fps:.1f}')
        self.get_logger().info(f' Hedef FPS: {self.target_fps:.1f}')
        self.get_logger().info(f' Loop: {"Aktif" if self.loop_video else "Pasif"}')
        self.get_logger().info(f' Topic: /camera/image_raw')
        self.get_logger().info('═' * 70)
    
    def timer_callback(self):
        """Her timer tick'inde çalışır - frame okur ve publish eder"""
        
        if not self.video_capture or not self.video_capture.isOpened():
            return
        
        # TODO: Videodan bir frame oku
        ret, frame = self.video_capture.read()
        
        if not ret:
            # Video bitti
            if self.loop_video:
                # Başa sar
                self.get_logger().info('Video başa sarılıyor')
                self.video_capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
                self.current_frame = 0
                ret, frame = self.video_capture.read()
            else:
                # Bitir
                self.get_logger().info(' Video tamamlandı!')
                self.destroy_node()
                return
        
        if ret and frame is not None:
            self.current_frame += 1
            
            # TODO: BGR'den RGB'ye çevir (ROS standart RGB kullanır)
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # TODO: OpenCV frame'i ROS Image mesajına dönüştür
            msg = self.bridge.cv2_to_imgmsg(frame_rgb, encoding='rgb8')
            msg.header.stamp = self.get_clock().now().to_msg()  # Zaman damgası
            msg.header.frame_id = 'camera_link'  # Çerçeve kimliği
            
            # TODO: Topic'e publish et
            self.image_publisher.publish(msg)
            
            # TODO: Her 100 frame'de bir durum raporu
            if self.current_frame % 100 == 0:
                progress = (self.current_frame / self.total_frames) * 100
                self.get_logger().info(
                    f' Frame: {self.current_frame}/{self.total_frames} ({progress:.1f}%)')
    
    def destroy_node(self):
        """Node kapatılırken video'yu kapat"""
        if self.video_capture:
            self.video_capture.release()
            self.get_logger().info('Video Player Node kapatıldı')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        video_player = VideoPlayerNode()
        rclpy.spin(video_player)
    except KeyboardInterrupt:
        pass
    finally:
        if 'video_player' in locals():
            video_player.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
