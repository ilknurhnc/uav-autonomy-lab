# UAV Autonomy Lab — Autonomous UAV Software & Vision-Guided Flight

UAV Autonomy Lab; **PX4, Gazebo, MAVSDK, Python, C/C++ ve OpenCV** kullanılarak otonom görev, görüntü işleme ve uçuş kontrol sistemlerinin uçtan uca geliştirilmesini amaçlayan uygulamalı bir İHA yazılım projesidir.

Bu proje iki temel amaçla geliştirilmektedir:

1. Otonom İHA sistemlerinin sensörden uçuş kontrolüne kadar bütün yazılım zincirini uygulayarak öğrenmek.
2. Öğrendiğim konuları yalnızca teorik bilgi olarak değil; **çalışan kod, simülasyon, test, hata analizi ve teknik dokümantasyon** ile gösterebileceğim bir mühendislik portföyü oluşturmak.

Proje hazır bir otonomi sistemi kullanmak yerine aşamalı olarak geliştirilmektedir:

```text
Understand → Implement → Test → Debug → Validate → Document
```

---

# Güncel Proje Hedefi

Projenin güncel ana senaryosu, gerçek bir İHA görevine benzer uçtan uca bir otonom görev zinciri oluşturmaktır.

Hedef görev:

```text
Mission Start
      |
      v
Autonomous Takeoff
      |
      v
Waypoint Flight
      |
      v
Industrial / Factory Area
      |
      v
Autonomous Inspection Pattern
(U / S / Search Pattern)
      |
      v
Camera Search
      |
      v
Vehicle Detection
      |
      v
Target Tracking
      |
      v
Vision-Guided Control
      |
      v
Target Centering / Inspection
      |
      v
Continue Mission
      |
      v
Return To Launch
```

İlk geliştirme ve doğrulama PX4 SITL + Gazebo üzerinde yapılmaktadır.

Daha sonraki aşamalarda aynı görev mimarisinin **ArduPilot**, gerçek nesne tespiti, ROS2 ve Companion Computer donanımlarıyla genişletilmesi hedeflenmektedir.

---

# Şu Anda Çalışan Sistem

Projede şu ana kadar aşağıdaki temel yetenekler uygulanmış ve simülasyon ortamında test edilmiştir:

* PX4 SITL + Gazebo Harmonic simülasyonu
* X500 multicopter
* IMU, GPS, barometre ve magnetometer gibi sanal sensörler
* EKF2 durum tahmini
* MAVSDK bağlantısı
* Arm / Takeoff / Land
* Offboard Control
* Velocity setpoint kontrolü
* İleri uçuş ve kare rota denemeleri
* Gazebo kamera görüntüsünün Python'a aktarılması
* OpenCV gerçek zamanlı görüntü işleme
* HSV tabanlı renk tespiti
* Contour ve bounding box hesaplama
* Hedef merkezinin bulunması
* Görüntü merkezi ile hedef arasındaki `error_x` hesabı
* P Controller
* `error_x → yaw_speed` dönüşümü
* MAVSDK üzerinden PX4 yaw kontrolü
* İlk **closed-loop vision-guided flight** denemesi

---

# Vision-Guided Flight

Projede ulaşılan önemli kilometre taşlarından biri kamera verisinin doğrudan uçuş kontrolüne bağlanmasıdır.

Mevcut kontrol zinciri:

```text
Gazebo Camera
      |
      v
OpenCV
      |
      v
HSV Color Detection
      |
      v
Contour
      |
      v
Bounding Box
      |
      v
Target Center
      |
      v
error_x
      |
      v
P Controller
      |
      v
yaw_speed
      |
      v
MAVSDK Offboard
      |
      v
PX4
      |
      v
Vehicle Yaw
      |
      v
New Camera Frame
      |
      +-------- Feedback --------+
```

Bu yapı **closed-loop control** mantığını gerçek bir simülasyon üzerinde uygulamak için oluşturuldu.

Kontrol mantığı:

```text
error_x = target_x - image_center_x
```

Hedef görüntünün merkezinden uzaklaştıkça P Controller daha büyük yaw komutu üretir.

Hedef merkeze yaklaştıkça komut küçülür.

Belirlenen deadband içerisine girildiğinde:

```text
yaw_speed = 0
```

yapılarak gereksiz titreşim ve sürekli küçük düzeltmeler azaltılır.

Bu aşamada yapay zekâ yerine bilinçli olarak klasik görüntü işleme kullanılmaktadır. Amaç önce **pixel → detection → error → controller → vehicle response** zincirini anlamaktır.

---

# Öğrenilen Mühendislik Konuları

Bu proje yalnızca çalışan kod üretmek için değil, kullanılan sistemlerin altında ne olduğunu anlamak için geliştirilmektedir.

## Flight & Autonomy

* PX4 architecture
* SITL
* Offboard Control
* MAVLink
* MAVSDK
* Vehicle telemetry
* Coordinate systems
* Velocity commands
* Yaw / yaw rate
* Autonomous mission logic

## Computer Vision

* NumPy image representation
* Pixel coordinates
* BGR / RGB / HSV
* Color thresholding
* Binary masks
* Contours
* Bounding boxes
* Target center calculation
* Tracking error
* Real-time image processing

## Control

* Closed-loop control
* Feedback
* P Controller
* Controller gain (`Kp`)
* Deadband
* Command saturation
* Error → actuator command ilişkisi

## Python / Software Architecture

* Python modules
* Async programming
* `asyncio`
* Callbacks
* Threads
* Locks
* Shared state
* Modular project architecture

## PX4 Internals

* uORB
* EKF2
* Sensor processing
* Local position estimation
* Vehicle health
* Preflight checks
* Gazebo ↔ PX4 sensor pipeline

---

# Sistem Mimarisi

```text
                     AUTONOMY / MISSION
                            |
                            v
                        DECISION
                            |
             +--------------+--------------+
             |                             |
             v                             v
         VISION                        TELEMETRY
             |                             |
             v                             |
       Target Detection                    |
             |                             |
             v                             |
       Tracking Error                      |
             |                             |
             +--------------+--------------+
                            |
                            v
                     Motion Controller
                            |
                            v
                      MAVSDK / MAVLink
                            |
                            v
                           PX4
                            |
                 +----------+----------+
                 |          |          |
                 v          v          v
                EKF2    Controllers   uORB
                            |
                            v
                       Gazebo X500
                            |
                            v
                         Sensors
                            |
                         Feedback
```

PX4 düşük seviyeli uçuş kontrolü, stabilizasyon ve durum tahmininden sorumludur.

Companion Computer tarafı ise:

* görüntü işleme,
* hedef tespiti,
* görev yönetimi,
* karar verme,
* yüksek seviyeli hareket komutları

gibi görevleri üstlenmektedir.

---

# Karşılaşılan Problemler ve Debugging

Projede yalnızca başarılı sonuçlar değil, karşılaşılan teknik problemlerin kök nedenleri de incelenmektedir.

## Magnetometer / Heading Problemi

Custom Gazebo world geliştirildikten sonra PX4:

```text
Strong magnetic interference
no heading reference
```

hataları verdi ve araç arm edilemedi.

İncelemede EKF2'nin magnetometer verisine güvenmediği ve yaw alignment gerçekleştiremediği görüldü.

Sorunun custom world içerisinde Gazebo system plugin'lerinin manuel tanımlanması ile PX4'ün kendi `server.config` plugin yönetiminin çakışmasından kaynaklandığı tespit edildi.

Magnetometer için gerekli Gazebo Harmonic compatibility ayarları:

```xml
<use_units_gauss>true</use_units_gauss>
<use_earth_frame_ned>true</use_earth_frame_ned>
```

uygulandı ve custom world içerisindeki gereksiz system plugin tanımları kaldırıldı.

Sonuç:

```text
Preflight check: OK
```

Bu hata üzerinden:

```text
Symptom
   |
   v
Sensor Data
   |
   v
EKF State
   |
   v
Plugin Configuration
   |
   v
Root Cause
```

şeklinde sistematik hata ayıklama yaklaşımı uygulandı.

## Python / Gazebo Environment Problemi

Companion uygulamasında:

```text
ModuleNotFoundError: No module named 'cv2'
```

ve daha sonra:

```text
ModuleNotFoundError: No module named 'gz'
```

hatalarıyla karşılaşıldı.

Problemin farklı Python virtual environment'larının ve Homebrew Gazebo Python binding'lerinin farklı konumlarda bulunmasından kaynaklandığı tespit edildi.

Companion uygulamaları için proje `.venv` ortamı kullanılmaya başlandı ve Gazebo Python binding path'leri environment üzerinden tanımlandı.

Bu süreçte:

* Virtual environment
* Python interpreter
* `PYTHONPATH`
* Dynamic library path
* Dependency isolation

konuları pratik olarak incelendi.

Daha ayrıntılı hata kayıtları:

```text
docs/troubleshooting.md
```

dosyasında tutulmaktadır.

---

# Repository Yapısı

```text
uav-autonomy-lab/
|
├── companion/
|   |
|   ├── autonomy/
|   |   └── motion_controller.py
|   |
|   ├── missions/
|   |   ├── takeoff_land.py
|   |   ├── offboard_forward.py
|   |   ├── offboard_square.py
|   |   └── vision_tracking.py
|   |
|   ├── telemetry/
|   |   └── telemetry_monitor.py
|   |
|   └── vision/
|       └── camera_viewer.py
|
├── simulation/
|   └── worlds/
|       └── vision_test.sdf
|
├── scripts/
|   ├── start_simulation.sh
|   └── stop_simulation.sh
|
├── docs/
|   ├── architecture.md
|   └── troubleshooting.md
|
├── requirements.txt
└── README.md
```

---

# Development Roadmap

## Phase 1 — PX4 + Gazebo Simulation — Completed

* PX4 SITL
* Gazebo Harmonic
* X500
* Sensor pipeline
* EKF2
* Local Position

## Phase 2 — MAVSDK & Telemetry — In Progress

* MAVSDK connection
* Position
* Velocity
* Altitude
* Heading
* Vehicle health
* Async telemetry

## Phase 3 — Offboard Flight Control — In Progress

* Arm / Takeoff / Land
* Velocity setpoints
* Forward flight
* Square mission
* Yaw control
* Offboard safety

## Phase 4 — Gazebo Camera Pipeline — Completed

* Simulated camera
* Gazebo image topic
* Python subscriber
* OpenCV integration
* Real-time frame processing

## Phase 5 — Computer Vision Foundations — Completed

* BGR / HSV
* Color thresholding
* Mask
* Contour
* Bounding box
* Target center
* Tracking error

## Phase 6 — Target Tracking & Vision Control — Current

Completed:

* Red target detection
* Target center calculation
* Horizontal tracking error
* P Controller
* Error → yaw speed
* MAVSDK Offboard integration
* Initial closed-loop yaw test

Next:

* Target-loss behavior
* Search behavior
* Tracking stability
* PD Controller

## Phase 7 — Autonomous Mission

Hedef:

```text
TAKEOFF
   |
   v
WAYPOINT FLIGHT
   |
   v
ARRIVE AT FACTORY
   |
   v
SEARCH
   |
   v
DETECT
   |
   v
TRACK
   |
   v
CONTINUE MISSION
   |
   v
RTL
```

Bu aşamada yaklaşık 2 km'lik simüle edilmiş görev uçuşu ve görev bölgesine otonom erişim geliştirilecektir.

## Phase 8 — Industrial Inspection Environment

Gazebo üzerinde daha gerçekçi:

* fabrika,
* endüstriyel alan,
* araçlar,
* yapılar,
* engeller

içeren görev ortamı oluşturulacaktır.

İHA bu ortam üzerinde U / S benzeri inspection pattern uygulayacaktır.

## Phase 9 — Vehicle Detection

Renk tabanlı test hedefi gerçek nesne tespitine dönüştürülecektir.

Önce klasik Computer Vision yöntemleri değerlendirilecek, ardından gerektiğinde:

* YOLO
* Confidence score
* IoU
* NMS
* Real-time inference

kullanılacaktır.

## Phase 10 — Mission State Machine

```text
INIT
 |
 v
TAKEOFF
 |
 v
NAVIGATE
 |
 v
SEARCH
 |
 v
TARGET_DETECTED
 |
 v
TRACK
 |
 v
TARGET_CENTERED
 |
 v
CONTINUE
 |
 v
RTL
 |
 v
LAND
```

Failsafe, target loss, communication loss ve vehicle health kontrolleri bu mimariye eklenecektir.

## Phase 11 — ArduPilot

PX4 üzerinde geliştirilen görev mimarisinin ArduPilot SITL üzerinde de uygulanması ve iki flight stack arasındaki farkların incelenmesi hedeflenmektedir.

## Phase 12 — ROS2

Sistem ilerleyen aşamada:

```text
camera_node
     |
     v
vision_node
     |
     v
autonomy_node
     |
     v
control_node
     |
     v
PX4 / ArduPilot
```

şeklinde ROS2 tabanlı bir mimariye genişletilecektir.

## Phase 13 — Ground Control Station

Telemetri, görev durumu, hedef bilgisi ve uçuş verilerini görüntüleyen basit bir Ground Control Station geliştirilecektir.

## Phase 14 — Multi-UAV / Swarm

Tek araç mimarisi tamamlandıktan sonra:

* Multiple SITL vehicles
* Vehicle ID
* Multi-UAV telemetry
* Task allocation
* Formation
* Cooperative missions

konuları incelenecektir.

## Phase 15 — Edge Deployment

Son aşamalarda sistem:

* NVIDIA Jetson
* Raspberry Pi
* Linux Companion Computer

gibi gerçek donanımlara taşınacaktır.

---

# Development Approach

Her özellik mümkün olduğunca aşağıdaki mühendislik döngüsüyle geliştirilmektedir:

```text
Understand
    |
    v
Implement
    |
    v
Simulate
    |
    v
Test
    |
    v
Observe
    |
    v
Debug
    |
    v
Validate
    |
    v
Document
```

Amaç yalnızca çalışan bir demo oluşturmak değil; **neden çalıştığını, hata verdiğinde sistemin hangi katmanında sorun olduğunu ve bileşenlerin birbirleriyle nasıl haberleştiğini anlayabilmektir.**

---

# Long-Term Goal

Projenin uzun vadeli hedefi aşağıdaki uçtan uca sistemi gerçekleştirmektir:

```text
Autonomous Takeoff
        |
        v
Mission Navigation
        |
        v
Industrial Area Search
        |
        v
Vehicle Detection
        |
        v
Target Tracking
        |
        v
Vision-Guided Flight
        |
        v
Mission Decision
        |
        v
Return To Launch
```

Bu proje geliştikçe yalnızca yeni özellikler eklenmeyecek; **mimari, testler, hata kayıtları ve teknik dokümantasyon da düzenli olarak güncellenecektir.**
