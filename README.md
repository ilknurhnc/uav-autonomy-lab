# UAV Autonomy Lab — İHA Otonomi Laboratuvarı

PX4, Gazebo, MAVSDK, Python, C++, OpenCV, ağ programlama (Networking), yapay zekâ, ROS2 ve çoklu İHA sistemlerini kullanarak **uçtan uca bir otonom İHA yazılım sistemi geliştirmek ve bu sistemin her katmanını öğrenmek** amacıyla oluşturulmuş uygulamalı bir projedir.

Bu projenin amacı yalnızca otonom uçuş gerçekleştiren bir İHA geliştirmek değildir. Aynı zamanda sensörden başlayarak görüntü işleme, karar verme, haberleşme ve uçuş kontrolüne kadar bütün yazılım zincirinin nasıl çalıştığını mühendislik seviyesinde anlamaktır.

Proje aşamalı olarak geliştirilecektir. Her yeni özellik önce öğrenilecek, ardından uygulanacak, simülasyonda test edilecek ve mevcut sisteme entegre edilecektir.

---

# 🎯 Proje Hedefleri

Bu proje kapsamında aşağıdaki yeteneklerin geliştirilmesi hedeflenmektedir:

* PX4 tabanlı otonom uçuş
* Gazebo üzerinde İHA simülasyonu
* Companion Computer mimarisi
* MAVLink / MAVSDK haberleşmesi
* Gerçek zamanlı telemetri
* Offboard uçuş kontrolü
* Kamera tabanlı algılama
* OpenCV ile hedef tespiti ve takibi
* Görüntü tabanlı otonom uçuş
* Ağ üzerinden telemetri ve veri haberleşmesi
* Otonom görev yönetimi
* Yapay zekâ tabanlı nesne tespiti
* ROS2 entegrasyonu
* Yer Kontrol İstasyonu (GCS) geliştirilmesi
* Çoklu İHA ve sürü sistemleri
* Jetson / Raspberry Pi gibi Companion Computer sistemlerine deployment

---

# 🏗️ Sistem Mimarisi

Projenin uzun vadeli hedef mimarisi:

```text
                         KAMERA
                            │
                            ▼
                    GÖRÜNTÜ İŞLEME
                   OpenCV / AI / YOLO
                            │
                            ▼
                       HEDEF TESPİTİ
                            │
                            ▼
                       HEDEF KONUMU
                            │
                            ▼
                       OTONOMİ SİSTEMİ
                  Görev / State Machine
                            │
                            ▼
                       HAREKET KONTROLÜ
                            │
                            ▼
                      MAVSDK / MAVLink
                            │
                       UDP / NETWORK
                            │
                            ▼
                           PX4
                  ┌─────────┼─────────┐
                  │         │         │
                 EKF2   Controllers  uORB
                  │         │
                  └────┬────┘
                       ▼
                  Gazebo X500
                       │
                       ▼
                 Sanal Sensörler
```

PX4, İHA'nın düşük seviyeli uçuş kontrolü ve stabilizasyonundan sorumludur.

Companion Computer ise yüksek seviyeli görevlerden sorumludur:

* Görüntü işleme
* Hedef tespiti
* Karar verme
* Görev yönetimi
* Networking
* Yüksek seviyeli otonom hareket komutları

---

# 🔄 Mevcut PX4 Veri Akışı

Simülasyon ortamında aşağıdaki veri akışı doğrulanmıştır:

```text
Gazebo Physics
      │
      ▼
Sanal Sensörler
IMU / GPS / Barometre / Manyetometre
      │
      ▼
PX4 Gazebo Bridge
      │
      ▼
PX4 Sensor Processing
      │
      ▼
uORB
sensor_combined
      │
      ▼
EKF2
Durum Tahmini
      │
      ▼
uORB
vehicle_local_position
      │
      ▼
PX4 Flight Controllers
```

Bu yapı daha ileri seviye otonom uçuş algoritmaları için temel oluşturmaktadır.

---

# ✅ Şu Ana Kadar Gerçekleştirilenler

## 1. PX4 ve Gazebo Simülasyonu

* PX4 SITL kurulumu
* Apple Silicon macOS üzerinde PX4 derleme
* Gazebo Harmonic kurulumu
* X500 İHA simülasyonu
* PX4 ↔ Gazebo bağlantısı
* Sanal sensör veri akışı
* IMU verilerinin doğrulanması
* Gyroscope ve accelerometer verilerinin uORB üzerinden incelenmesi
* EKF2 local position tahmininin doğrulanması

---

## 2. Companion Computer Altyapısı

Aşağıdaki alanlar için modüler bir Companion Computer yapısı oluşturulmuştur:

```text
companion/

├── autonomy/
├── missions/
├── telemetry/
└── vision/
```

Python; görüntü işleme, telemetri ve yüksek seviyeli otonomi geliştirmelerinde kullanılacaktır.

C++ ise ilerleyen aşamalarda PX4 modülleri, performans gerektiren işlemler ve daha düşük seviyeli sistem geliştirmelerinde kullanılacaktır.

---

## 3. MAVSDK ve Uçuş Denemeleri

MAVSDK kullanılarak ilk uçuş uygulamaları oluşturulmuştur.

Mevcut çalışmalar:

* PX4 bağlantısı
* Arm / Disarm
* Takeoff
* Landing
* Offboard Control
* İleri hareket
* Kare rota uçuşu

Bu modüller otonomi sistemi geliştikçe genişletilecektir.

---

## 4. Kamera Altyapısı

Gazebo simülasyon ortamına kamera sistemi eklenmiştir.

Simüle edilen kamera görüntülerinin Python tarafına aktarılması için gerekli altyapı oluşturulmuştur.

Bu kamera sistemi ilerleyen aşamalarda:

* OpenCV
* Hedef tespiti
* Hedef takibi
* YOLO
* Vision-Guided Flight

çalışmalarının temelini oluşturacaktır.

---

# 🔵 Şu Anki Geliştirme Aşaması

## Phase 5 — Computer Vision Temelleri

Şu anda projenin ana geliştirme konusu görüntü işlemedir.

Öğrenilecek veri akışı:

```text
Kamera Görüntüsü
       │
       ▼
NumPy Array
       │
       ▼
Pixel
       │
       ▼
BGR / HSV
       │
       ▼
Color Mask
       │
       ▼
Contour
       │
       ▼
Bounding Box
       │
       ▼
Hedef Merkezi
       │
       ▼
Error X / Error Y
```

İlk hedefimiz yapay zekâ kullanmadan basit bir görsel hedefi güvenilir şekilde tespit edebilmektir.

---

# 🗺️ Geliştirme ve Öğrenme Yol Haritası

## Phase 1 — PX4 + Gazebo Simülasyonu ✅

* PX4 SITL
* Gazebo Harmonic
* X500
* Sanal sensörler
* uORB
* EKF2
* Local Position

---

## Phase 2 — MAVSDK ve Telemetri 🟡

* MAVSDK bağlantısı
* Position telemetry
* Velocity telemetry
* Altitude
* Heading
* Battery
* Flight Mode
* Connection monitoring
* Async telemetry streams

---

## Phase 3 — Offboard Flight Control 🟡

* Arm / Disarm
* Takeoff / Landing
* Position setpoint
* Velocity setpoint
* İleri hareket
* Kare rota
* Offboard güvenliği
* Komut doğrulama

---

## Phase 4 — Gazebo Kamera Pipeline 🟡

* Simüle edilmiş İHA kamerası
* Gazebo camera topics
* Frame alma
* Python entegrasyonu
* OpenCV entegrasyonu
* Gerçek zamanlı görüntü işleme

---

## Phase 5 — Computer Vision Temelleri 🔵

**Şu anda bulunduğumuz aşama.**

Öğrenilecek konular:

* NumPy görüntü yapısı
* `frame.shape`
* Pixel kavramı
* Görüntü koordinat sistemi
* BGR
* RGB
* HSV
* Color Thresholding
* Binary Mask
* Contour
* Bounding Box
* Hedef merkezi hesaplama

---

## Phase 6 — Hedef Tespiti ve Takibi

```text
Kamera
   │
   ▼
Hedef Tespiti
   │
   ▼
Bounding Box
   │
   ▼
Hedef Merkezi
   │
   ▼
Error X / Error Y
```

Öğrenilecek konular:

* Renk tabanlı hedef tespiti
* Hedef konumunun bulunması
* Görüntü merkezinin hesaplanması
* Tracking Error
* Hedef kaybolduğunda yapılacak işlemler
* Basit Target Tracking

---

## Phase 7 — Networking ve Gerçek Zamanlı Veri Haberleşmesi

Networking ayrı bir teorik ders olarak değil, doğrudan İHA sistemleri üzerinde öğrenilecektir.

### Temel konular

* IP adresi
* Port
* localhost / `127.0.0.1`
* Client / Server
* TCP
* UDP
* Socket programlama
* UDP Datagram
* JSON
* Serialization
* Async Networking
* Gerçek zamanlı veri aktarımı

Örnek sistem:

```text
           İHA / SIMULATION

           altitude
           position
           battery
           heading
           mission state
                │
                ▼
           Serialization
                │
                ▼
            UDP Socket
                │
                ▼
              NETWORK
                │
                ▼
       GROUND CONTROL STATION
```

Ayrıca MAVLink'in UDP üzerinden nasıl taşındığı incelenecektir.

---

## Phase 8 — Vision-Guided Flight

Görüntü işleme sistemi ile uçuş kontrol sistemi birleştirilecektir.

```text
Camera
   │
   ▼
Target Detector
   │
   ▼
Target Center
   │
   ▼
Tracking Error
   │
   ▼
Controller
   │
   ▼
MAVSDK
   │
   ▼
MAVLink
   │
   ▼
PX4
   │
   ▼
İHA
```

Öğrenilecek konular:

* Closed-loop control
* P Controller
* Velocity commands
* Yaw control
* Target centering
* Offboard control
* Control-loop frequency
* Hedef kaybolması

---

## Phase 9 — Otonom Görev Mimarisi

İHA'nın hangi durumda ne yapacağını belirleyen bir görev yönetim sistemi geliştirilecektir.

```text
INIT
 │
 ▼
TAKEOFF
 │
 ▼
MISSION
 │
 ▼
SEARCH
 │
 ▼
TARGET_DETECTED
 │
 ▼
TARGET_TRACKING
 │
 ▼
RTH
 │
 ▼
LAND
```

Öğrenilecek konular:

* State Machine
* Mission Manager
* Search
* Target Tracking
* Return To Home
* Failsafe
* Battery kontrolü
* GPS kontrolü
* EKF health
* Communication loss

---

## Phase 10 — Yapay Zekâ ve YOLO

Klasik görüntü işleme öğrenildikten sonra AI tabanlı hedef tespitine geçilecektir.

Öğrenilecek konular:

* YOLO
* Object Detection
* Bounding Box
* Confidence Score
* IoU
* Non-Maximum Suppression
* Detection vs Tracking
* Real-time inference

```text
Camera
   │
   ▼
YOLO
   │
   ▼
Detection
   │
   ▼
Tracker
   │
   ▼
Autonomy
```

---

## Phase 11 — ROS2

Mevcut sistem daha sonra ROS2 mimarisine taşınacaktır.

Öğrenilecek konular:

* Node
* Topic
* Publisher
* Subscriber
* Message
* Service
* Launch files

Örnek:

```text
camera_node
     │
     ▼
/camera/image
     │
     ▼
vision_node
     │
     ▼
/target
     │
     ▼
autonomy_node
     │
     ▼
/control_command
     │
     ▼
px4_interface_node
```

---

## Phase 12 — Ground Control Station

Kendi basit Yer Kontrol İstasyonumuz geliştirilecektir.

Görüntülenecek bilgiler:

* İHA konumu
* Harita
* İrtifa
* Hız
* Heading
* Battery
* Flight Mode
* Mission State
* Hedef bilgileri
* Waypoint'ler
* Bağlantı durumu

---

## Phase 13 — Multi-UAV / Swarm

Tek İHA mimarisi tamamlandıktan sonra sistem birden fazla İHA'ya genişletilecektir.

```text
               SWARM MANAGER

              /      |      \
             /       |       \
          UAV-1    UAV-2    UAV-3
            │        │        │
           PX4      PX4      PX4
```

Öğrenilecek konular:

* Multiple PX4 SITL
* Vehicle ID
* Multi-UAV telemetry
* Inter-UAV communication
* Swarm Manager
* Formation
* Task Allocation
* Cooperative Missions

---

## Phase 14 — Edge Deployment

Projenin son aşamalarında yazılım gerçek Companion Computer donanımlarına taşınacaktır.

Hedef platformlar:

* NVIDIA Jetson
* Raspberry Pi
* Linux tabanlı Companion Computer

Öğrenilecek konular:

* Linux deployment
* CPU / GPU
* Performance
* Latency
* Camera interface
* ONNX
* TensorRT
* Real-time inference optimization

---

# 📁 Repository Yapısı

Mevcut proje yapısı:

```text
uav-autonomy-lab/
│
├── companion/
│   │
│   ├── autonomy/
│   │   └── yüksek seviyeli otonomi ve hareket kontrolü
│   │
│   ├── missions/
│   │   └── uçuş görevleri ve Offboard deneyleri
│   │
│   ├── telemetry/
│   │   └── telemetri ve araç durum takibi
│   │
│   └── vision/
│       └── kamera ve görüntü işleme
│
├── docs/
│   └── architecture.md
│
├── scripts/
│   ├── start_simulation.sh
│   └── stop_simulation.sh
│
├── simulation/
│   └── worlds/
│
├── requirements.txt
└── README.md
```

Yeni klasörler yalnızca ilgili geliştirme aşamasına gerçekten geçildiğinde oluşturulacaktır.

Böylece README'de varmış gibi görünen fakat gerçekte bulunmayan modüllerin oluşması engellenecektir.

---

# ⚙️ Geliştirme Prensipleri

## 1. Çalışan sistemi koru

Teknik bir sebep olmadıkça çalışan kod yeniden yazılmayacaktır.

## 2. Küçük adımlarla geliştir

```text
ANLA
 │
 ▼
UYGULA
 │
 ▼
TEST ET
 │
 ▼
DOĞRULA
 │
 ▼
COMMIT
 │
 ▼
DOKÜMANTE ET
```

## 3. Sorumlulukları ayır

Aşağıdaki sistemler birbirinden mantıksal olarak ayrılacaktır:

* Perception
* Autonomy
* Missions
* Telemetry
* Networking
* Flight Control

## 4. Düşük seviyeli uçuş kontrolü PX4'ın görevidir

Companion Computer doğrudan motor RPM hesaplamayacaktır.

```text
Autonomy
   │
   ▼
Position / Velocity / Yaw Setpoint
   │
   ▼
PX4
   │
   ▼
Flight Controller
   │
   ▼
Control Allocation
   │
   ▼
Motors
```

## 5. Önce anla, sonra soyutla

Yeni class, modül, klasör veya abstraction yalnızca neden gerekli olduğu anlaşıldığında eklenecektir.

---

# 🧠 Öğrenme Hedefleri

Bu repository aynı zamanda uygulamalı bir **Autonomous Systems Engineering Lab** olarak kullanılacaktır.

## Programlama

* Python
* C++
* Async Programming
* Modular Software Design
* Exception Handling
* Real-time Programming Concepts

## İHA Sistemleri

* PX4
* SITL
* Gazebo
* MAVLink
* MAVSDK
* uORB
* EKF2
* Offboard Control
* Telemetry

## Computer Vision

* NumPy
* OpenCV
* Image Processing
* Target Detection
* Target Tracking
* Camera Coordinate Systems

## Networking

* TCP/IP
* UDP
* Socket
* Serialization
* Real-time Telemetry
* Distributed Systems

## Yapay Zekâ

* Object Detection
* YOLO
* Tracking
* Model Deployment

## Robotik

* ROS2
* Publisher / Subscriber
* State Machine
* Autonomous Mission Systems

## İleri Seviye İHA Sistemleri

* Ground Control Station
* Multi-UAV
* Swarm Coordination
* Edge Computing

---

# 🧩 Projenin Temel Felsefesi

Amaç yalnızca İHA'yı otonom olarak uçurmak değildir.

Amaç aşağıdaki zincirin tamamını anlamaktır:

```text
SENSÖR
   │
   ▼
VERİ
   │
   ▼
DURUM TAHMİNİ
   │
   ▼
ALGILAMA
   │
   ▼
KARAR
   │
   ▼
KONTROL KOMUTU
   │
   ▼
HABERLEŞME
   │
   ▼
PX4
   │
   ▼
FLIGHT CONTROLLER
   │
   ▼
İHA
```

Projede kullanılan her önemli sistem mümkün olduğunca:

**öğrenilecek → uygulanacak → test edilecek → doğrulanacak → dokümante edilecektir.**

---

# 🚀 Şu Anki Hedef

**Phase 5 — Computer Vision Temelleri**

İlk hedef:

```text
Gazebo Kamera
      │
      ▼
Python
      │
      ▼
NumPy Frame
      │
      ▼
OpenCV
      │
      ▼
Pixel Yapısını Anla
      │
      ▼
Hedefi Tespit Et
      │
      ▼
Hedef Konumunu Hesapla
```

Bir sonraki kilometre taşı, Gazebo simülasyon ortamında bulunan görsel bir hedefin OpenCV kullanılarak güvenilir şekilde tespit edilmesidir.
