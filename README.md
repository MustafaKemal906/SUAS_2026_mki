# Pipline 

## 1) Waypointlerin Üretilmesi

Öncelikle `waypoint py` klasöründeki `waypoints_final.py` dosyası yürütülür ve output olarak:

> mission_waypoints.json

dosyası elde edilir. Bu bizim haritalama algoritmasıyla oluşturulmuş yol planımızdır. 
## 2) Image bekleme

Sonra ise `/image recieve & send/image_waiter.py` dosyası yürütülür. Bu dosya server'a fotoğraf akmasını bekler ve drone waypointlere ulaştığında server'a yollanan fotoğrafları teker teker alarak  `camera_images/` klasörü altında kaydeder.

## 3) Mission

ilk olarak `cpp - drone controller/` projesi *build* klasörü açılarak içine build edilir. Daha sonra ilk etapta ürettiğimiz *mission_waypoints.json* dosyası da build içine yerleştirilir. Sonra görevi yüklemek için *build* konumunda `./mission` şeklinde görev drone gönderilir ve drone görevleri tamamlamaya ve çetktiği fotoğrafları server'a yollamaya başlar. İkinci etapta bahsettiğimiz dosya ise fotoğrafları kaydeder.

## 4) ODM

Tüm fotoğraflar toplandıktan sonra 

```  bash
$ docker run -ti -p 3000:3000 opendronemap/nodeodm
```

gibi bir komutla ODM başlatılır. 

Sonra ise `/image recieve & send/send.py` yürütülür ve `camera_images/` altında toplanan fotoğraflar ODM'e gönderilir. Böylece harita oluşturulur.

# Images 

Yapılan haritalama testlerinin outputları `mapping tests img/` klasörü altında part by part izlenebilir.