import cv2
import pyzed.sl as sl
import numpy as np



##Kusbakisi algoritmaları ici köse noktaları secilir.
##Bu script ile kameraya baglanip köse noktalari rahatca secilir
## program bitince kordinalar terminale basilir
## arabayı yolun ortasına koyduktan sonra baslatilir ve bir kare alır, 
# sürekli yeni görüntü takibine gerek kalmaz bu yüzden referans noktasi icin araba iyi yerlestirilmeli 
# Nokta etiketleri


point_labels = ["Sol Üst", "Sol Alt", "Sağ Üst", "Sağ Alt"]
clicked_points = []

# Mouse callback fonksiyonu 
def mouse_callback(event, x, y, flags, param):   
    if event == cv2.EVENT_LBUTTONDOWN and len(clicked_points) < 4:   ## bu fonksiyon mouse evente baglaniyor 4 noktaya tıklandı mi diye kontrol ediyor
        label = point_labels[len(clicked_points)]      
        print(f"{label} noktası seçildi: ({x}, {y})")    ## x ve y kordinatları basiliyor
        clicked_points.append((x, y))    

# ZED kamera başlat
zed = sl.Camera()
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720  ##cozunurluk ayari
init_params.depth_mode = sl.DEPTH_MODE.NONE

if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
    print("ZED kamera açılamadı.")
    exit()

runtime_parameters = sl.RuntimeParameters()
image = sl.Mat()

cv2.namedWindow("ZED Görüntü")
cv2.setMouseCallback("ZED Görüntü", mouse_callback)  ##yukarıdaki callback fonskiyonu bagladik

print("\nLütfen sırasıyla aşağıdaki noktaları seçin:")
for label in point_labels:
    print(f" {label}")

while True:
    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:    ##temel goruntu islemleri görüntüyü alıyor mu diye bakıyor 
        zed.retrieve_image(image, sl.VIEW.LEFT) ## zeddeki sol kamerayı aldı
        frame = image.get_data()   ###resmi OpenCV'ye uygun data tipine donstrdu
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)
        frame_rgb = cv2.resize( frame_rgb,(640,480))

        # Seçilen noktaları çiz ve etiketle
        for i, point in enumerate(clicked_points):
            cv2.circle(frame_rgb, point, 5, (0, 0, 255), -1)
            cv2.putText(frame_rgb, point_labels[i], (point[0] + 5, point[1] - 5),   ## sectigimiz noktaları düzgünce algılamak icin çember ve resim koyuyor
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)    

        cv2.imshow("ZED Görüntü", frame_rgb)

        # 4 nokta seçildiyse döngüyü bitir
        if len(clicked_points) == 4:
            print("\n4 nokta seçildi. Program kapanıyor...\n")
            break

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Pencereleri kapat ve kamerayı kapat
cv2.destroyAllWindows()
zed.close()

# Terminale noktaları yazdır
print("Seçilen Noktalar:")
for i, pt in enumerate(clicked_points):
    print(f"{point_labels[i]}: {pt}")
