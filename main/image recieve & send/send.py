import os
import requests

# NodeODM adresi (Docker çalışıyor olmalı)
nodeodm_url = 'http://localhost:3000/task/new/init'
upload_url = 'http://localhost:3000/task/new/upload/'
commit_url = 'http://localhost:3000/task/new/commit/'

# 1. Görevi Başlat
response = requests.post(nodeodm_url, data={'name': 'gazebo_project'})
task_id = response.json()['uuid']
print(f"Görev oluşturuldu ID: {task_id}")

# 2. Resimleri Yükle
image_dir = "./camera_images"
images = [os.path.join(image_dir, f) for f in os.listdir(image_dir) if f.endswith('.jpg')]

if not images:
    print("Hata: Yüklenecek resim bulunamadı!")
else:
    for img_path in images:
        with open(img_path, 'rb') as f:
            print(f"Yükleniyor: {img_path}")
            requests.post(f"{upload_url}{task_id}", files={'images': f})

    # 3. İşlemeyi Başlat (Sadece en hızlı 2D ayarları)
    options = {
        'fast-orthophoto': True,      # Sadece 2D ortofoto üretir, 3D işlemlerini atlar
        'dsm': False,                 # Yüzey modelini (Sayısal Yüzey Modeli) kapatır
        'orthophoto-resolution': 2.0  # cm/pixel çözünürlük
    }
    
    requests.post(f"{commit_url}{task_id}", json=options)
    print("İşleme başladı! http://localhost:3000 adresinden takip edebilirsin.")
