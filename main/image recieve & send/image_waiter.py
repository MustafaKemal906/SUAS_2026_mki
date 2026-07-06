import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
import os
import piexif
from fractions import Fraction

# C++ ile haberleşmek için gereken ağ (Network) kütüphaneleri
import socket
import threading

class TopicData(Node):

    def __init__(self):
        super().__init__('topic_data')
        
        # --- Image subscription ---
        self.subscription = self.create_subscription(
            Image,
            '/world/baylands/model/x500_mono_cam_0/link/camera_link/sensor/imager/image', 
            self.image_handler,
            10)

        # --- GPS subscription ---
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/gps',
            self.gps_callback,
            qos_profile_sensor_data) # GPS için mecburi QoS profili

        # --- Variables ---
        self.image_count = 0
        self.current_gps = None
        
        # Tetikleme bayrağı (Trigger flag) - C++'tan sinyal gelince True olacak
        self.take_photo = False 

        self.directory = "./camera_images" 
        if not os.path.exists(self.directory):
            os.makedirs(self.directory)

        # --- UDP Dinleyici İş Parçacığı (Thread) Başlatma ---
        self.udp_thread = threading.Thread(target=self.udp_listener, daemon=True)
        self.udp_thread.start()

        self.get_logger().info('📸 Sistem başlatıldı. C++ kodundan UDP (Port: 5005) tetiklemesi bekleniyor...')

    # C++ kodunun gönderdiği UDP paketlerini dinleyen fonksiyon
    def udp_listener(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('127.0.0.1', 5005))
        
        while True:
            # Gelen veriyi bekler (bloklar)
            data, addr = sock.recvfrom(1024)
            message = data.decode('utf-8').strip()
            
            if message == "SNAP":
                self.take_photo = True # Kameraya fotoğraf çekme emri veriyoruz
                self.get_logger().info('⚡ C++ tetiklemesi (SNAP) alındı! Fotoğraf çekiliyor...')

    def gps_callback(self, msg):
        self.current_gps = msg

    def decimal_to_dms_rational(self, degrees):
        deg = int(abs(degrees))
        min_float = (abs(degrees) - deg) * 60
        minutes = int(min_float)
        sec_float = (min_float - minutes) * 60
        return ((deg, 1), (minutes, 1), (int(sec_float * 100000), 100000))

    def exif_handler(self, h, w, _):
        lat = self.current_gps.latitude
        lon = self.current_gps.longitude
        alt = self.current_gps.altitude
                
        lat_dms = self.decimal_to_dms_rational(lat)
        lon_dms = self.decimal_to_dms_rational(lon)
        lat_ref = 'N' if lat >= 0 else 'S'
        lon_ref = 'E' if lon >= 0 else 'W'
                
        alt_fraction = Fraction(str(alt)).limit_denominator(10000)
        alt_rational = (alt_fraction.numerator, alt_fraction.denominator)

        gps_ifd = {
            piexif.GPSIFD.GPSLatitudeRef: lat_ref,
            piexif.GPSIFD.GPSLatitude: lat_dms,
            piexif.GPSIFD.GPSLongitudeRef: lon_ref,
            piexif.GPSIFD.GPSLongitude: lon_dms,
            piexif.GPSIFD.GPSAltitudeRef: 0,
            piexif.GPSIFD.GPSAltitude: alt_rational
        }
                
        zeroth_ifd = {
            piexif.ImageIFD.Make: b"Gazebo",
            piexif.ImageIFD.Model: b"X500_Sim_Cam",
        }
                
        exif_ifd = {
            piexif.ExifIFD.FocalLength: (8, 1),             
            piexif.ExifIFD.FocalLengthIn35mmFilm: 24,       
            piexif.ExifIFD.PixelXDimension: w,              
            piexif.ExifIFD.PixelYDimension: h               
        }
                
        exif_dict = {"0th": zeroth_ifd, "Exif": exif_ifd, "GPS": gps_ifd}
        return piexif.dump(exif_dict)

    def image_name(self):
        image_name = f"image_{self.image_count:04d}.jpg"
        self.image_count += 1
        return os.path.join(self.directory, image_name)

    # ANA FONKSİYON (Sürekli çalışan kamera dinleyicisi)
    def image_handler(self, msg):
        try:
            # Eğer C++'tan tetikleme emri (SNAP) gelmediyse fonksiyonu sonlandır, resmi yoksay
            if not self.take_photo:
                return

            # EĞER GELDİYSE: Bir sonraki tetiklemeye kadar tekrar resim kaydetmemek için bayrağı indir
            self.take_photo = False

            if self.current_gps is None:
                self.get_logger().warning('Tetikleme alındı ancak henüz GPS verisi yok! Atlanıyor.')
                return

            image_bridge = CvBridge().imgmsg_to_cv2(msg, "bgr8")
            h, w, _ = image_bridge.shape
            target_path = self.image_name()

            cv2.imwrite(target_path, image_bridge)
            exif_data = self.exif_handler(h, w, _)
            piexif.insert(exif_data, target_path)

            self.get_logger().info(f'✅ Başarılı: {target_path} kaydedildi (İrtifa: {self.current_gps.altitude:.2f}m)')

        except Exception as e:
            self.get_logger().error(f'Hata: {e}')


def main(args=None):
    rclpy.init(args=args)
    subscriber = TopicData()
    
    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()