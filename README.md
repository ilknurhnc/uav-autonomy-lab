# UAV Autonomy Lab — Autonomous UAV Software & Vision-Guided Flight

UAV Autonomy Lab; **PX4, Gazebo, MAVSDK, Python, C/C++, OpenCV ve LiDAR** kullanılarak otonom görev, görüntü işleme, çevre algılama ve uçuş kontrol sistemlerinin uçtan uca geliştirilmesini amaçlayan uygulamalı bir İHA yazılım projesidir.

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
Environment Perception
(Camera + LiDAR)
      |
      v
Obstacle Detection / Clustering
      |
      v
Autonomous Inspection / Search
      |
      v
Target Detection
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

İlk geliştirme ve doğrulama **PX4 SITL + Gazebo** üzerinde yapılmaktadır.

Güncel kısa vadeli hedef, kırmızı test hedefinin doğrudan görünür olmadığı bir sahnede LiDAR ile çevredeki engelleri ayrı objeler olarak algılayıp bu bilgiyi daha sonra otonom arama davranışına bağlamaktır.

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
* HSV tabanlı kırmızı hedef tespiti
* Contour ve bounding box hesaplama
* Hedef merkezinin bulunması
* Görüntü merkezi ile hedef arasındaki `error_x` hesabı
* P Controller
* `error_x → yaw_speed` dönüşümü
* MAVSDK üzerinden PX4 yaw kontrolü
* `SEARCH → TRACK → CENTERED` hedef takip davranışı
* Kamera görüş alanı dışındaki hedefi yaw taramasıyla bulma ve merkezleme
* Gazebo 2D LiDAR verisinin Python'a aktarılması
* `LaserScan` mesajlarının işlenmesi
* Geçersiz LiDAR ölçümlerinin (`inf`, `nan`, `<= 0`) filtrelenmesi
* Her LiDAR noktası için mesafe, açı ve scan index bilgisinin çıkarılması
* Ardışık LiDAR noktalarının spatial cluster'lara ayrılması
* Aynı mesafeye yakın fakat farklı açılardaki engellerin ayrı objeler olarak tutulması
* Cluster başına ortalama mesafe, merkez açısı, açısal genişlik ve nokta sayısı hesabı
* İlk **closed-loop vision-guided flight**
* İlk **LiDAR obstacle clustering** testi

---

# Vision-Guided Flight

Projede ulaşılan önemli kilometre taşlarından biri kamera verisinin doğrudan uçuş kontrolüne bağlanmasıdır.

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

Hedef kamera görüş alanında değilken sistem `SEARCH` durumunda çevreyi tarar. Hedef algılandığında `TRACK` durumuna geçer. Hedef deadband içerisine girdiğinde `CENTERED` durumu oluşur.

```text
SEARCH
   |
   | target detected
   v
TRACK
   |
   | abs(error_x) < deadband
   v
CENTERED
```

Bu davranış Gazebo simülasyonunda hedef başlangıç görüş alanının dışına taşınarak test edilmiş ve drone'un hedefi bulup merkezlemesi doğrulanmıştır.

Bu aşamada yapay zekâ yerine bilinçli olarak klasik görüntü işleme kullanılmaktadır. Amaç önce:

```text
pixel
  ↓
detection
  ↓
error
  ↓
controller
  ↓
vehicle response
```

zincirini anlamaktır.

---

# LiDAR Obstacle Detection & Clustering

Projeye çevre algılama yeteneği kazandırmak amacıyla Gazebo üzerinde **2D LiDAR pipeline** geliştirilmeye başlandı.

LiDAR verisi şu zincir üzerinden işlenmektedir:

```text
Gazebo 2D LiDAR
      |
      v
LaserScan Message
      |
      v
Raw Ranges
      |
      v
Invalid Measurement Filtering
(inf / nan / <= 0)
      |
      v
Valid LiDAR Points
(distance + angle + index)
      |
      v
Scan Segmentation
      |
      v
Spatial Clusters
      |
      v
Obstacle Information
(distance / center angle / angular width)
```

## LiDAR Nokta Temsili

Her geçerli LiDAR ölçümü Python tarafında bir dictionary ile temsil edilmektedir:

```python
{
    "distance": distance,
    "angle": angle,
    "index": i,
}
```

Burada:

* `distance` → LiDAR noktasının sensöre olan mesafesi
* `angle` → noktanın LiDAR taramasındaki açısı
* `index` → noktanın `LaserScan` içerisindeki sıra numarası

LiDAR ışınının açısı:

```text
angle = angle_min + index * angle_step
```

şeklinde hesaplanmaktadır.

Kullanılan veri yapısı:

```text
Dictionary
    |
    +--> Tek LiDAR noktası
         distance + angle + index


List of dictionaries
    |
    +--> Tek obje / tek cluster
         Aynı engele ait LiDAR noktaları


List of lists
    |
    +--> Bütün cluster'lar
         Sahnede algılanan bütün engel grupları
```

Kod tarafındaki isimler:

```text
point
→ tek LiDAR noktası

current_cluster
→ şu anda oluşturulmakta olan tek obje / cluster

clusters
→ tamamlanmış bütün obje / cluster listesi
```

## Clustering Mantığı

Ham LiDAR verisinde yalnızca en yakın mesafeyi seçmek yeterli değildir.

Aynı objenin farklı kenarlarında çok küçük ölçüm farklılıkları oluşabildiği gibi, birbirinden farklı iki obje yaklaşık aynı mesafede de bulunabilir.

Bu nedenle ardışık LiDAR noktaları şu iki kriterle karşılaştırılmaktadır:

```python
distance_difference <= DISTANCE_JUMP_THRESHOLD
and
index_difference <= INDEX_GAP_THRESHOLD
```

Kullanılan başlangıç parametreleri:

```python
DISTANCE_JUMP_THRESHOLD = 1.0
INDEX_GAP_THRESHOLD = 2
MIN_CLUSTER_POINTS = 3
```

Anlamları:

`DISTANCE_JUMP_THRESHOLD`

Ardışık iki LiDAR noktasının mesafeleri arasındaki izin verilen maksimum farktır.

`INDEX_GAP_THRESHOLD`

LiDAR taramasındaki iki nokta arasındaki izin verilen maksimum scan index boşluğudur.

`MIN_CLUSTER_POINTS`

Bir nokta grubunun gerçek bir obstacle cluster olarak kabul edilmesi için gereken minimum LiDAR noktası sayısıdır.

Algoritmanın temel mantığı:

```text
Yeni LiDAR noktası
        |
        v
Önceki cluster'ın son noktası
        |
        v
Mesafe farkını hesapla
        |
        v
Index farkını hesapla
        |
        v
Mesafe yakın mı?
VE
Index yakın mı?
     /        \
   EVET       HAYIR
    |           |
    v           v
Aynı cluster   Önceki cluster'ı kapat
append(point)  Yeni cluster başlat
```

Bu yöntem sayesinde birbirine yakın mesafede bulunan fakat LiDAR scan üzerinde farklı bölgelerde bulunan engeller ayrı cluster'lar olarak tutulabilmektedir.

## Doğrulanan Test Sonucu

Gazebo test sahnesinde aynı LiDAR taramasında dört ayrı cluster kararlı şekilde algılandı.

Örnek çıktı:

```text
Detected clusters: 4

Cluster 1:
Distance=20.23 m
Center Angle=-22.1 deg
Angular Width=-27.2..-17.1 deg
Points=41

Cluster 2:
Distance=29.78 m
Center Angle=-5.8 deg
Angular Width=-6.1..-5.4 deg
Points=4

Cluster 3:
Distance=29.78 m
Center Angle=5.8 deg
Angular Width=5.4..6.1 deg
Points=4

Cluster 4:
Distance=20.23 m
Center Angle=22.1 deg
Angular Width=17.1..27.2 deg
Points=41
```

Bu testte özellikle yaklaşık aynı mesafedeki iki farklı engelin tek obje olarak birleştirilmemesi sağlandı.

Bu, yalnızca:

```text
minimum distance
```

kullanmak yerine:

```text
distance continuity
+
scan index continuity
```

kullanılmasının avantajını gösterdi.

---

# LiDAR Veri Yapısının Python Tarafı

Bu özellik geliştirilirken Python tarafında aşağıdaki veri yapıları uygulandı.

## Dictionary

Tek bir LiDAR noktasını tutar.

```python
{
    "distance": 20.2,
    "angle": -0.38,
    "index": 102
}
```

## List of Dictionaries

Aynı objeye ait LiDAR noktalarını tutar.

```python
current_cluster = [
    {"distance": 20.1, "angle": -0.40, "index": 100},
    {"distance": 20.2, "angle": -0.39, "index": 101},
    {"distance": 20.3, "angle": -0.38, "index": 102},
]
```

## List of Lists

Algılanan bütün objeleri / cluster'ları tutar.

```python
clusters = [
    cluster_1,
    cluster_2,
    cluster_3,
]
```

Yapının tamamı:

```text
clusters
|
├── Cluster 1
|   ├── point
|   |   ├── distance
|   |   ├── angle
|   |   └── index
|   |
|   └── point
|
├── Cluster 2
|   ├── point
|   └── point
|
└── Cluster 3
    ├── point
    └── point
```

Bu yapı ilerleyen aşamada yalnızca terminal çıktısı üretmek yerine görev karar sistemine obstacle verisi sağlayacaktır.

---

# LiDAR Sonraki Hedef

Bir sonraki amaç LiDAR'ın yalnızca:

```text
"Burada 4 obje var."
```

demesi değildir.

Amaç bu bilgiyi drone'un karar mekanizmasına bağlamaktır.

```text
Target Not Visible
      |
      v
LiDAR Scan
      |
      v
Detect Separate Obstacles
      |
      v
Determine Obstacle Position
      |
      v
Generate Inspection Viewpoint
      |
      v
Move Around Obstacle
      |
      v
Camera Search
      |
      v
Target Detected?
   /        \
 NO         YES
 |           |
 v           v
Next       TRACK
Obstacle     |
             v
          CENTERED
```

Böylece drone, hedefin hangi objenin arkasında olduğunu önceden bilmeden çevresini sistematik olarak araştırabilecektir.

Mevcut clustering yöntemi bilinçli olarak basit bir **1D LaserScan segmentation** yaklaşımıdır.

İlerleyen aşamada gerekirse LiDAR noktaları:

```text
x = r * cos(theta)
y = r * sin(theta)
```

ile Kartezyen koordinatlara dönüştürülecek ve daha geometrik clustering yöntemleri değerlendirilecektir.

---

# Öğrenilen Mühendislik Konuları

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
* SEARCH / TRACK / CENTERED state logic

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

## LiDAR / Environment Perception

* Gazebo `LaserScan` messages
* LiDAR range measurements
* `angle_min`
* `angle_step`
* Scan indexing
* `inf` / `nan` filtering
* Polar sensor measurements
* LiDAR point representation
* Sequential scan processing
* Distance continuity
* Index continuity
* Spatial clustering
* Cluster size filtering
* Average obstacle distance
* Obstacle center angle
* Angular width estimation

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
* Lists
* Dictionaries
* Nested data structures
* `.append()`
* List slicing: `[1:]`
* Positive indexing: `[0]`
* Negative indexing: `[-1]`
* `enumerate()`
* `len()`
* `sum()`
* Generator expressions
* `abs()`
* `math.isinf()`
* `math.isnan()`
* `math.degrees()`
* `if / else`
* `continue`
* Callbacks
* Threads
* Locks
* `asyncio`
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
               +----------------+----------------+
               |                                 |
               v                                 v
            VISION                            LIDAR
               |                                 |
               v                                 v
       Target Detection                  LaserScan Processing
               |                                 |
               v                                 v
       Tracking Error                    Obstacle Clusters
               |                                 |
               +----------------+----------------+
                                |
                                v
                         Mission / Search Logic
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
                       +--------+--------+
                       |                 |
                       v                 v
                    Camera            LiDAR
                       |                 |
                       +--------+--------+
                                |
                             Feedback
```

PX4 düşük seviyeli uçuş kontrolü, stabilizasyon ve durum tahmininden sorumludur.

Companion Computer tarafı ise:

* görüntü işleme
* LiDAR verisi işleme
* obstacle clustering
* hedef tespiti
* görev yönetimi
* karar verme
* yüksek seviyeli hareket komutları

gibi görevleri üstlenmektedir.

---

# Karşılaşılan Problemler ve Debugging

## Magnetometer / Heading Problemi

Custom Gazebo world geliştirildikten sonra PX4:

```text
Strong magnetic interference
no heading reference
```

hataları verdi ve araç arm edilemedi.

İncelemede EKF2'nin magnetometer verisine güvenmediği ve yaw alignment gerçekleştiremediği görüldü.

Sorunun custom world içerisinde Gazebo system plugin'lerinin manuel tanımlanması ile PX4'ün kendi `server.config` plugin yönetiminin çakışmasından kaynaklandığı tespit edildi.

Gerekli Gazebo Harmonic compatibility ayarları:

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

ve:

```text
ModuleNotFoundError: No module named 'gz'
```

hatalarıyla karşılaşıldı.

Problemin farklı Python virtual environment'larının ve Homebrew Gazebo Python binding'lerinin farklı konumlarda bulunmasından kaynaklandığı tespit edildi.

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
|       ├── camera_viewer.py
|       └── lidar_viewer.py
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

## Phase 6 — Target Tracking & Vision Control — Completed

Completed:

* Red target detection
* Target center calculation
* Horizontal tracking error
* P Controller
* Error → yaw speed
* MAVSDK Offboard integration
* Closed-loop yaw tracking
* `SEARCH → TRACK → CENTERED` state logic
* Target outside initial camera view test

Further improvements:

* Tracking stability
* PD Controller
* More robust target-loss handling

## Phase 7 — LiDAR Obstacle Perception — Current

Completed:

* Gazebo 2D LiDAR integration
* `LaserScan` subscriber
* Raw range processing
* `inf` / `nan` filtering
* Angle calculation from scan index
* LiDAR point dictionaries
* Sequential scan segmentation
* Distance continuity checks
* Index continuity checks
* Minimum cluster point filtering
* Multiple obstacle clustering
* Average distance calculation
* Center angle calculation
* Angular width calculation
* Same-distance separate obstacle validation

Next:

* Return obstacle information to mission logic instead of only printing
* Vehicle pose / heading integration
* Convert LiDAR observations into local/world coordinates
* Generate safe inspection viewpoints
* Inspect different sides of detected obstacles
* Integrate LiDAR search with camera target detection

## Phase 8 — Autonomous Mission

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
LIDAR PERCEPTION
   |
   v
INSPECT OBSTACLES
   |
   v
DETECT TARGET
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

## Phase 9 — Industrial Inspection Environment

Gazebo üzerinde daha gerçekçi:

* fabrika
* endüstriyel alan
* araçlar
* yapılar
* engeller

içeren görev ortamı oluşturulacaktır.

İHA bu ortam üzerinde U / S benzeri inspection pattern uygulayacaktır.

## Phase 10 — Vehicle Detection

Renk tabanlı test hedefi gerçek nesne tespitine dönüştürülecektir.

Gerektiğinde:

* YOLO
* Confidence score
* IoU
* NMS
* Real-time inference

konuları uygulanacaktır.

## Phase 11 — Mission State Machine

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
PERCEIVE
 |
 v
INSPECT
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

## Phase 12 — ArduPilot

PX4 üzerinde geliştirilen görev mimarisinin ArduPilot SITL üzerinde de uygulanması ve iki flight stack arasındaki farkların incelenmesi hedeflenmektedir.

## Phase 13 — ROS2

```text
camera_node
     |
     v
vision_node

lidar_node
     |
     v
perception_node
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

## Phase 14 — Ground Control Station

Telemetri, görev durumu, hedef bilgisi ve uçuş verilerini görüntüleyen basit bir Ground Control Station geliştirilecektir.

## Phase 15 — Multi-UAV / Swarm

Tek araç mimarisi tamamlandıktan sonra:

* Multiple SITL vehicles
* Vehicle ID
* Multi-UAV telemetry
* Task allocation
* Formation
* Cooperative missions

konuları incelenecektir.

## Phase 16 — Edge Deployment

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

```text
Autonomous Takeoff
        |
        v
Mission Navigation
        |
        v
Industrial Area Perception
(Camera + LiDAR)
        |
        v
Obstacle-Aware Search
        |
        v
Dynamic Inspection Viewpoints
        |
        v
Vehicle / Target Detection
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
