import json
import os
from info import Info

def info_to_geojson_feature(info, name="gorev", description="Tanımsız Nokta", local_x=0.0, local_y=0.0, filename="cikti.json"):
    # Eklenecek yeni özellik (feature)
    new_feature = {
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

    # Dosya varsa oku, yoksa yeni oluştur
    if os.path.exists(filename) and os.path.getsize(filename) > 0:
        with open(filename, "r", encoding="utf-8") as f:
            geojson_data = json.load(f)
    else:
        geojson_data = {
            "type": "FeatureCollection",
            "features": []
        }

    # Yeni veriyi listeye ekle
    geojson_data["features"].append(new_feature)

    # Güncellenmiş içeriği dosyaya yaz
    with open(filename, "w", encoding="utf-8") as f:
        json.dump(geojson_data, f, ensure_ascii=False, indent=2)

def main():
    a = Info()
    info_to_geojson_feature(a)

if __name__ == "__main__":
    main()
