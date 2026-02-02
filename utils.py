import cv2
from math import radians, cos, sin


#Todo: Işın gönderme algoritması
#Todo: Belirli açılarla ışınlar gönderir ve kırmızı çizgilerle gösterir
#Todo: Bu sayede çevredeki engelleri tespit eder
def turnInfo(frame, leftLineAngle, rightLineAngle):
    #Todo: Sadece yolun göründüğü kısmı almamıza yarar
    x1, x2 = 60, 500
    y1, y2 = 300, 1080

    roi = frame[y1:y2, :]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    mean_frame = cv2.mean(gray)[0]
    _, thresh = cv2.threshold(gray, 1.6*float(mean_frame), 255, cv2.THRESH_BINARY)

    h, w = roi.shape[:2]
    center_y = y2 - y1

    red_line_counts = {"sol": 0, "sag": 0}


    #Todo: yarım darilerik bir tarama yapar
    #Todo: Kaç derecede bir ışın atılacağını kontrol edersin
    for angle in range(0, 185):  # 0'dan 184'e kadar
        if 90 <= angle < 185 and (angle - 90) % leftLineAngle == 0:
            label = "sol"
            center_x = (x1 + x2) // 2
        elif 0 <= angle < 90 and angle % rightLineAngle == 0:
            label = "sag"
            center_x = 1200
        else:
            continue  # Adım uyumsuzsa geç

        #Todo: Işının çıkış noktası
        center = (center_x, center_y)
        #Todo:Işının hareketini belirler
        theta = radians(angle)
        dx = cos(theta)
        dy = sin(theta)
        hit = False
        for i in range(1, 1000):
            xi = int(center[0] + dx * i)
            yi = int(center[1] - dy * i)

            if xi < 0 or xi >= w or yi < 0 or yi >= h:
                break
            #Todo: Beyaz piksel kontrolü
            if thresh[yi, xi] == 255:
                cv2.line(roi, center, (xi, yi), (0, 255, 0), 1) #Todo: Evetse yeşil çiz
                hit = True
                break
        #Todo: Bulunmadıysa burada kırmızı çiz
        if not hit:
            xi = int(center[0] + dx * 300)
            yi = int(center[1] - dy * 300)
            if 0 <= xi < w and 0 <= yi < h:
                cv2.line(roi, center, (xi, yi), (0, 0, 255), 1)
                red_line_counts[label] += 1

        cv2.circle(roi, center, 5, (255, 0, 255), -1)

    # Metin ekleme
    cv2.putText(roi, f"Sol merkez: {red_line_counts['sol']}", (50, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    cv2.putText(roi, f"Sag merkez: {red_line_counts['sag']}", (50, 100),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    cv2.imshow("180 Derecelik Işın Analizi (Tek Forlu)", roi)

    return red_line_counts["sol"], red_line_counts["sag"]


#Todo: Aracın konum bilgilerini harita formatına çevirir
def info_to_geojson_feature(info, name="gorev", description="Tanımsız Nokta", local_x=0.0, local_y=0.0):
    return {
        "type": "Feature",
        "properties": {
            "name": name,
            "description": description,
            "local_x": local_x,
            "local_y": local_y,
            "heading": info.get_imu()
        },
        "geometry": {
            "type": "Point",
            "coordinates": info.get_gps()
        }
    }

