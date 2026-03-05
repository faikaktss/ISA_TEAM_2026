#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime


class VideoRecorderNode(Node):
    def __init__(self):
        super().__init__('video_recorder_node')
        
        # Parametreler
        self.declare_parameter('output_path', '/videos/output')
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('record_duration', 30.0)  # saniye (0 = sınırsız)
        
        output_path = self.get_parameter('output_path').value
        self.fps = self.get_parameter('fps').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.record_duration = self.get_parameter('record_duration').value
        
        # Çıktı dizinini oluştur
        os.makedirs(output_path, exist_ok=True)
        
        # Dosya adı (timestamp ile)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.output_file = os.path.join(output_path, f'lane_detection_test_{timestamp}.mp4')
        
        # Video writer
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.video_writer = cv2.VideoWriter(
            self.output_file,
            fourcc,
            self.fps,
            (self.width, self.height)
        )
        
        # ROS bridge
        self.bridge = CvBridge()
        
        # Abone oluştur - debug image'i kaydet
        self.image_sub = self.create_subscription(
            Image,
            '/lane/debug_image',
            self.image_callback,
            10
        )
        
        # Açı ve offset değerlerini al (overlay için)
        self.angle_sub = self.create_subscription(
            Float32,
            '/lane/angle',
            self.angle_callback,
            10
        )
        
        self.offset_sub = self.create_subscription(
            Float32,
            '/lane/offset',
            self.offset_callback,
            10
        )
        
        # Değişkenler
        self.current_angle = 0.0
        self.current_offset = 0.0
        self.frame_count = 0
        self.start_time = self.get_clock().now()
        
        # Süre limiti varsa timer oluştur
        if self.record_duration > 0:
            self.timer = self.create_timer(self.record_duration, self.stop_recording)
        
        self.get_logger().info('VIDEO RECORDER NODE BAŞLATILDI')
        self.get_logger().info(f'  Çıktı: {self.output_file}')
        self.get_logger().info(f'  FPS: {self.fps}')
        self.get_logger().info(f'  Çözünürlük: {self.width}x{self.height}')
        if self.record_duration > 0:
            self.get_logger().info(f'   Süre: {self.record_duration} saniye')
        else:
            self.get_logger().info(f'   Süre: Sınırsız (Ctrl+C ile durdur)')
    
    def angle_callback(self, msg):
        self.current_angle = msg.data
    
    def offset_callback(self, msg):
        self.current_offset = msg.data
    
    def image_callback(self, msg):
        try:
            # ROS Image -> OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            # Çözünürlüğü ayarla
            if frame_bgr.shape[1] != self.width or frame_bgr.shape[0] != self.height:
                frame_bgr = cv2.resize(frame_bgr, (self.width, self.height))
            
            # Overlay ekle (açı ve offset bilgisi)
            self.add_overlay(frame_bgr)
            
            # Video'ya yaz
            self.video_writer.write(frame_bgr)
            self.frame_count += 1
            
            # Her 100 frame'de rapor
            if self.frame_count % 100 == 0:
                elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
                self.get_logger().info(
                    f'📹 Frame: {self.frame_count} | '
                    f'Süre: {elapsed:.1f}s | '
                    f'Açı: {self.current_angle:.1f}° | '
                    f'Offset: {self.current_offset:.0f}'
                )
        
        except Exception as e:
            self.get_logger().error(f'Frame yazma hatası: {str(e)}')
    
    def add_overlay(self, frame):
        """Frame üzerine açı ve offset bilgisi ekle"""
        # Yarı saydam siyah panel
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (400, 120), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
        
        # Metin bilgileri
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(frame, f'Aci: {self.current_angle:.1f} derece', 
                    (20, 50), font, 0.8, (0, 255, 0), 2)
        cv2.putText(frame, f'Offset: {self.current_offset:.0f} piksel', 
                    (20, 90), font, 0.8, (0, 255, 0), 2)
        
        # Frame sayısı
        cv2.putText(frame, f'Frame: {self.frame_count}', 
                    (frame.shape[1] - 200, 40), font, 0.6, (255, 255, 255), 2)
    
    def stop_recording(self):
        """Kayıt durdur"""
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        self.get_logger().info('KAYIT TAMAMLANDI!')
        self.get_logger().info(f'  Dosya: {self.output_file}')
        self.get_logger().info(f'  Toplam Frame: {self.frame_count}')
        self.get_logger().info(f'   Süre: {elapsed:.1f} saniye')
        self.get_logger().info(f'  Ortalama FPS: {self.frame_count/elapsed:.1f}')
        
        # Video writer'ı kapat
        self.video_writer.release()
        
        # Node'u durdur
        self.destroy_node()
        rclpy.shutdown()
    
    def __del__(self):
        """Cleanup"""
        if hasattr(self, 'video_writer'):
            self.video_writer.release()


def main(args=None):
    rclpy.init(args=args)
    recorder = VideoRecorderNode()
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.get_logger().info('Ctrl+C ile durduruldu')
        recorder.stop_recording()
    except Exception as e:
        recorder.get_logger().error(f'Hata: {str(e)}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()















