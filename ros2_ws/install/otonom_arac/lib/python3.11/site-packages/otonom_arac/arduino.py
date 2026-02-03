import serial
from time import sleep


# Todo: COM = hangi porttan bağlanacak , baudrate = iletişim hızı
# Todo: Bilgisayardan gelen komutu donanım seviyesinde uygulamaya yarar
#Todo: Aracı hareket ettiren beyin gibi çalışır
class Teensy:
    def __init__(self,COM='COM3',baudrate=9600,timeout=1):
        # Todo: Seri portu başlat
        #Todo: Com3 portunu 9600 baudrate ile aç, 1 saniye timeout ile
        self.ser = serial.Serial(COM,baudrate,timeout=timeout)
    
    #Todo: Teensy'e veri gönderir
    def setValue(self,value):
        self.ser.write(value.encode())
    
     
    def getValue(self):
        # Todo: önceki eski verileri temizle
        self.ser.reset_input_buffer()
        data = self.ser.readline().decode(errors="ignore").strip()
        return data
    

#Todo: Arduino ile iletişim kurar
#Todo: Encoder'dan mesafe verisi alır
#Todo: teensy gibi çalışır ama daha yüksek baudrate ile
#Todo: Tekerleği kaç mm döndüğünü ölçer 
class Arduino:
    def __init__(self,COM='COM4',baudrate=115200,timeout=1):
        self.ser = serial.Serial(COM,baudrate,timeout=timeout)
        self.ilk_mesafe = None


    def setValue(self,value):
        if value is not None:
            value = str(value)
            self.ser.reset_output_buffer()
            self.ser.write(value.encode())
        
    def getValue(self):
        self.ser.reset_input_buffer()
        data = self.ser.readline().decode(errors="ignore").strip()
        return data
    
    # Todo: Encoder'dan mesafe verisi alır
    def encoder_distance(self):
        self.setValue("1000")
        distance = self.getValue()
        if(distance is not None and distance !=''):
            return int(distance)
        else:
            print("deger yok")

            return None
    
# ar = Arduino()
# while(1):


#     ar.setValue("1000")
#     print(ar.getValue())
    

# import serial
# from time import time

# class Arduino:
#     def __init__(self, COM='COM9', baudrate=9600, timeout=1):
#         self.ser = serial.Serial(COM, baudrate, timeout=timeout)

#     def setValue(self, value):
#         self.ser.write(value.encode())

#     def getValue(self):
#         return self.ser.readline().decode(errors="ignore").strip()

#     def get_distance_until_target(self, target_distance=100, timeout=5):
#         """
#         1000 gönderilir, mesafe verileri alınır.
#         Mesafe belirlenen değerin altına inince 2000 gönderilir.
#         """
#         self.setValue("1000")
#         print("Arduino'ya '1000' gönderildi — Veri toplanıyor...")

#         start_time = time()
#         response = None

#         while time() - start_time < timeout:
#             if self.ser.in_waiting > 0:
#                 data = self.getValue()
#                 if data:
#                     try:
#                         distance = int(data)
#                         print(f"Mesafe: {distance}")

#                         if distance < target_distance:
#                             print(f"Hedef mesafe ({target_distance}) ulaşıldı.")
#                             self.setValue("2000")
#                             print("Arduino'ya '2000' gönderildi — Veri alma durdu.")
#                             return distance
#                     except ValueError:
#                         print(f"Geçersiz veri: {data}")

#         print("Zaman aşımı — hedefe ulaşılamadı.")
#         self.setValue("2000")  # Yine de veri toplamayı durdur
#         return None

# class Teensy:
#     def __init__(self,COM='COM8',baudrate=9600,timeout=1):
#         self.ser = serial.Serial(COM,baudrate,timeout=timeout)
    
#     def setValue(self,value):
#         self.ser.write(value.encode())
     
#     def getValue(self):
#         data = self.ser.readline().decode(errors="ignore").strip()
#         return data
    
# arduino = Arduino()
# son_mesafe = arduino.get_distance_until_target(target_distance=80, timeout=5)

# if son_mesafe:
#     print(f"Son mesafe verisi: {son_mesafe}")
# else:
#     print("Hedef mesafeye ulaşılamadı.")
