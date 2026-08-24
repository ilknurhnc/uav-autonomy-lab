# UAV Autonomy Lab

A hands-on UAV autonomy project built to explore the complete software stack of an autonomous aerial vehicle — from flight control and simulation to companion computing, networking and computer vision.

The project currently uses **PX4 SITL** and **Gazebo Harmonic** to simulate an X500 quadrotor. Future stages will introduce a Python-based companion computer using **MAVSDK/MAVLink**, networking experiments, PX4 C++ development and OpenCV-based perception.

## Architecture

```text
                 ┌─────────────────────┐
                 │   Computer Vision   │
                 │       OpenCV        │
                 └──────────┬──────────┘
                            │
                            ▼
                 ┌─────────────────────┐
                 │ Companion Computer  │
                 │                     │
                 │ Python / C++        │
                 │ Mission Logic       │
                 │ Autonomy            │
                 │ MAVSDK              │
                 └──────────┬──────────┘
                            │
                         MAVLink
                       UDP / Serial
                            │
                            ▼
                 ┌─────────────────────┐
                 │        PX4          │
                 │                     │
                 │ uORB                │
                 │ EKF2                │
                 │ Flight Controllers  │
                 │ Actuator Control    │
                 └──────────┬──────────┘
                            │
                            ▼
                 ┌─────────────────────┐
                 │       Gazebo        │
                 │                     │
                 │ X500                │
                 │ Physics             │
                 │ Virtual Sensors     │
                 │ Motors              │
                 └─────────────────────┘
```

## Current Status

The first simulation milestone is complete.

- [x] PX4 SITL built successfully on Apple Silicon macOS
- [x] Gazebo Harmonic installed and configured
- [x] X500 quadrotor spawned in Gazebo
- [x] PX4 connected to the Gazebo simulation
- [x] Virtual IMU data received by PX4
- [x] Gyroscope and accelerometer data validated through uORB
- [x] EKF2 local position estimation validated
- [x] Gazebo and PX4 startup scripts added
- [ ] MAVSDK companion computer connection
- [ ] Telemetry monitoring from Python
- [ ] Autonomous takeoff and landing
- [ ] Waypoint missions
- [ ] Network communication experiments
- [ ] Custom PX4 C++ module
- [ ] Camera simulation
- [ ] OpenCV perception
- [ ] Vision-guided autonomous flight

## Verified Data Flow

The current simulation successfully implements:

```text
Gazebo Physics
      │
      ▼
Virtual Sensors
(IMU, GPS, Barometer, Magnetometer)
      │
      ▼
PX4 Gazebo Bridge
      │
      ▼
PX4 Sensor Processing
      │
      ▼
uORB: sensor_combined
      │
      ▼
EKF2
      │
      ▼
uORB: vehicle_local_position
```

Example IMU output observed from PX4:

```text
gyro_rad: [-0.00007, 0.00098, -0.00023]
accelerometer_m_s2: [0.00008, -0.00183, -9.78999]
```

Local position estimation was also successfully validated:

```text
xy_valid: True
z_valid: True
v_xy_valid: True
v_z_valid: True
dead_reckoning: False
```

## Repository Structure

```text
uav-autonomy-lab/
│
├── companion/
│   ├── autonomy/       # High-level autonomous behavior
│   ├── missions/       # Takeoff, landing and waypoint missions
│   ├── telemetry/      # PX4 telemetry monitoring
│   └── vision/         # OpenCV perception
│
├── networking/         # MAVLink/UDP and networking experiments
│
├── px4/
│   └── modules/        # Custom PX4 C++ modules
│
├── docs/               # Architecture and technical documentation
│
├── assets/             # Images and project media
│
└── scripts/
    ├── start_gazebo.sh
    └── start_px4.sh
```

## Running the Simulation

The macOS setup runs the Gazebo server and PX4 SITL as separate processes.

### 1. Start Gazebo

```bash
./scripts/start_gazebo.sh
```

### 2. Start PX4 SITL

Open another terminal:

```bash
./scripts/start_px4.sh
```

A successful PX4 startup should provide the PX4 shell:

```text
pxh>
```

Sensor data can then be inspected using:

```text
listener sensor_combined
```

and estimator output using:

```text
listener vehicle_local_position
```

## Companion Computer

The next stage introduces a separate high-level autonomy layer.

The intended architecture is:

```text
Python Application
       │
       ▼
     MAVSDK
       │
       ▼
     MAVLink
       │
       ▼
       PX4
```

The companion computer will be responsible for mission planning, telemetry processing, computer vision and high-level decision making.

PX4 remains responsible for real-time flight control, state estimation, stabilization and actuator control.

Later, this architecture can be transferred from simulation to hardware:

```text
Simulation                         Real UAV

Python                             Raspberry Pi / Jetson
   │                                      │
MAVSDK                                  MAVSDK
   │                                      │
MAVLink                                MAVLink
   │                                      │
PX4 SITL                          PX4 Flight Controller
   │                                      │
Gazebo                            ESCs + Motors + Sensors
```

## Planned Experiments

### Companion Computing
- MAVSDK connection
- Live telemetry
- Autonomous takeoff and landing
- Waypoint navigation
- Offboard control

### Networking
- MAVLink over UDP
- Packet inspection
- Port and endpoint configuration
- Communication between separate machines / virtual machines
- Simulated flight-controller and companion-computer network

### PX4 Internals
- uORB publish/subscribe architecture
- PX4 module structure
- Custom C++ module
- Internal topic communication
- Flight-control data flow

### Computer Vision
- Gazebo camera integration
- OpenCV frame processing
- Target detection
- Target position estimation
- Vision-based movement commands
- Closed-loop target tracking

## Goal

The goal of this repository is not simply to make a simulated drone fly.

It is intended to develop a practical understanding of how:

**simulation, sensors, state estimation, flight control, inter-process communication, networking, companion computing and perception**

fit together to form an autonomous UAV system.