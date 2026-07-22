#!/bin/bash
# This file shows the complete HAUV + QGC integration architecture as ASCII art

cat << 'EOF'

╔════════════════════════════════════════════════════════════════════════════╗
║                  HAUV + QGroundControl Integration                         ║
║                           System Architecture                              ║
╚════════════════════════════════════════════════════════════════════════════╝

LAYER 1: USER INTERFACE (Your PC)
═══════════════════════════════════════════════════════════════════════════
┌──────────────────────────────────────────────────────────────────────────┐
│                                                                            │
│  ┌─ QGroundControl (Windows/Linux/Mac) ───────────────────────────────┐  │
│  │                                                                     │  │
│  │  ┌─ Manual Control           ┌─ Telemetry Display               │  │  │
│  │  │ (Fly Screen)              │ (Attitude, Position, Health)     │  │  │
│  │  │ ✓ Joystick Input          │ ✓ 20 Hz updates                 │  │  │
│  │  │ ✓ Motor Control           │ ✓ Roll/Pitch/Yaw               │  │  │
│  │  │ ✓ Lights/Servo            │ ✓ Position & Velocity          │  │  │
│  │  └─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ───┴─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─────  │  │
│  │  ┌─ Analyzer                 ┌─ Settings                       │  │  │
│  │  │ (MAVLink Inspector)       │ (Connection Config)             │  │  │
│  │  │ ✓ Raw message view        │ ✓ UDP:14550 setup              │  │  │
│  │  │ ✓ Message rates (20 Hz)   │ ✓ Vehicle type selection       │  │  │
│  │  └─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─────┴─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─────  │  │
│  │                                                                     │  │
│  │  Your PC: 192.168.168.100                                          │  │
│  └─────────────────────────────────────────────────────────────────────┘  │
│                           │                                                 │
│                      UDP/MAVLink                                           │
│                 (Port 14550, Ethernet)                                     │
│                           │                                                 │
└───────────────────────────┼──────────────────────────────────────────────┘
                            │
                            │ Network: 192.168.168.0/24
                            │ (Ethernet or WiFi range)
                            │
LAYER 2: EMBEDDED MAIN COMPUTER (UP Board Linux System)
═══════════════════════════════════════════════════════════════════════════
┌───────────────────────────┼─────────────────────────────────────────────┐
│   ROS2 Foxy on Ubuntu     │                                             │
│                            ▼                                             │
│ ┌─────────────────────────────────────────────────────────────────┐    │
│ │         **mavlink_bridge_node** (NEW!)                          │    │
│ │                                                                  │    │
│ │  🔵 UDP Server (0.0.0.0:14550)                                 │    │
│ │     └─ Listens for QGC connections                            │    │
│ │     └─ Auto-discovers sender IP:port                          │    │
│ │                                                                  │    │
│ │  ✦ TELEMETRY STREAM (↑ to QGC)                                │    │
│ │  │                                                              │    │
│ │  ├─ HEARTBEAT (1 Hz)                                           │    │
│ │  │  └─ Vehicle type, armed state, flight mode                │    │
│ │  │                                                              │    │
│ │  ├─ ATTITUDE (20 Hz) from ROS2 topic /esp32/bno055_data       │    │
│ │  │  └─ Roll, Pitch, Yaw angles from BNO055 IMU               │    │
│ │  │                                                              │    │
│ │  ├─ GLOBAL_POSITION_INT (20 Hz) from /dvl/velocity_data       │    │
│ │  │  └─ Position (x,y,z), velocity, heading                   │    │
│ │  │  └─ Integrates DVL velocity over time                     │    │
│ │  │                                                              │    │
│ │  └─ SYS_STATUS (20 Hz) from /esp32/bar100_data                │    │
│ │     └─ Depth, pressure, temperature, battery                 │    │
│ │                                                                  │    │
│ │  ✦ COMMAND RECEIVE (↓ from QGC)                               │    │
│ │  │                                                              │    │
│ │  └─ MANUAL_CONTROL message                                    │    │
│ │     └─ Joystick input (X, Y, Z, R axes)                      │    │
│ │     └─ Convert: -1000..1000 → 1100..1900 µs PWM             │    │
│ │     └─ Publish to ROS2 /motor_data topic                    │    │
│ │                                                                  │    │
│ └─────────────────────────────────────────────────────────────────┘    │
│                            │           │ ROS2 Topics │                  │
│                            │           │             │                  │
│  ┌────────────────┐    ┌───▼─────────────────────────▼───┐             │
│  │ guidance_node  │    │ autopilot_pkg (existing)         │             │
│  ├────────────────┤    ├───────────────────────────────┤  │             │
│  │ Main control   │    │ ✓ guidance_node               │  │             │
│  │ logic          │───▶│ ✓ dvl_node                    │  │             │
│  │                │    │ ✓ camera_node (optional)      │  │             │
│  │ /motor_data ◀─┼───▶│                               │  │             │
│  │ /lights_data   │    │ Subscribes to:                │  │             │
│  │ /odometry      │    │ • /joy (joystick input)      │  │             │
│  │                │    │ • /esp32/bno055_data         │  │             │
│  │ Subscribes:    │    │ • /dvl/velocity_data         │  │             │
│  │ • /esp32/*     │    │ • /esp32/bar100_data         │  │             │
│  │ • /dvl/*       │    │                               │  │             │
│  │ • /joy         │    │ Publishes:                    │  │             │
│  │ • /bar100      │    │ • /motor_data                 │  │             │
│  └────────────────┘    │ • /lights_servo_data          │  │             │
│                        │ • /odometry                   │  │             │
│                        └───────────────┬───────────────┘  │             │
│                                        │                   │             │
│                        ┌───────────────────────┐           │             │
│                        │  JOY Node             │           │             │
│                        │  (PS4/Xbox/etc)       │           │             │
│                        │  /joy topic           │           │             │
│                        └───────────────────────┘           │             │
│                            │                               │             │
│   Micro-ROS Agent ────────┘                               │             │
│   (Serial Bridge)                                          │             │
│   /dev/ttyUSB0:115200                                     │             │
│                            │                               │             │
│   UP Board: 192.168.168.101                               │             │
└────────────────────────────┼───────────────────────────────┘
                            │
                    Serial: 115200 baud
                     /dev/ttyUSB0
                            │
LAYER 3: REAL-TIME MICROCONTROLLER (ESP32)
═══════════════════════════════════════════════════════════════════════════
┌───────────────────────────┼──────────────────────────────────────────────┐
│                           ▼                                               │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │  ESP32 WROOM (Micro-ROS firmware)                               │   │
│  │  rov_esp_main.ino                                               │   │
│  │                                                                   │   │
│  │  ✦ SENSOR INPUT (I2C/SPI)                                       │   │
│  │  │                                                                │   │
│  │  ├─ BNO055 (9-DOF IMU) @ 0x28                                  │   │
│  │  │  • Roll, Pitch, Yaw (fused orientation)                     │   │
│  │  │  • Published: /esp32/bno055_data (Twist msg)               │   │
│  │  │                                                                │   │
│  │  ├─ BAR100 (Depth & Pressure)                                  │   │
│  │  │  • Depth, Pressure, Temperature readings                   │   │
│  │  │  • Published: /esp32/bar100_data (Vector3 msg)            │   │
│  │  │                                                                │   │
│  │  ├─ [Optional] MPU6050 or other sensors                        │   │
│  │  │  • Extra gyro/accelerometer data                            │   │
│  │  │                                                                │   │
│  │  └─ External: DVL (Doppler Velocity Logger)                    │   │
│  │     └─ Ethernet connection (not ESP32 connected)              │   │
│  │     └─ Handled by separate dvl_node on UP Board              │   │
│  │                                                                   │   │
│  │  ✦ MOTOR OUTPUT (PWM)                                          │   │
│  │  │                                                                │   │
│  │  ├─ Motor 1: GPIO 16 (Thruster 1)                             │   │
│  │  ├─ Motor 2: GPIO 17 (Thruster 2)                             │   │
│  │  ├─ Motor 3: GPIO 18 (Thruster 3)                             │   │
│  │  ├─ Motor 4: GPIO 19 (Thruster 4)                             │   │
│  │  ├─ Motor 5: GPIO 20 (Thruster 5) - optional                 │   │
│  │  └─ Motor 6: GPIO 21 (Thruster 6) - optional                 │   │
│  │                                                                   │   │
│  │  ✦ ACTUATOR OUTPUT (PWM)                                       │   │
│  │  │                                                                │   │
│  │  ├─ Lights GPIO 22 (PWM for LED intensity)                    │   │
│  │  └─ Camera Servo GPIO 23 (Pan-tilt servo)                    │   │
│  │                                                                   │   │
│  │  ✦ MICRO-ROS COMMUNICATION                                     │   │
│  │  │                                                                │   │
│  │  └─ Serial UART (115200 baud)                                  │   │
│  │     ├─ Publishes sensor topics to ROS2                        │   │
│  │     └─ Subscribes to motor command topics                     │   │
│  │                                                                   │   │
│  └──────────────────────────────────────────────────────────────────┘   │
│                                                                            │
└────────────────────────────────────────────────────────────────────────┘


DATA FLOW EXAMPLES
═══════════════════════════════════════════════════════════════════════════

TELEMETRY PATH (ESP32 → QGC):
  ESP32 BNO055 
    ↓ I2C read (20 Hz)
  ESP32 IMU buffer
    ↓ Micro-ROS publish
  /esp32/bno055_data (ROS2 topic)
    ↓ ROS2 callback
  mavlink_bridge_node orientation_callback
    ↓ Convert to MAVLink ATTITUDE
  CRC calculation
    ↓ UDP send
  UDP packet → 192.168.168.100:14550
    ↓ Network transmission
  QGroundControl
    ↓ Parse & display
  Attitude Indicator (QGC Fly screen)

MOTOR COMMAND PATH (QGC → ESP32):
  User moves joystick in QGC
    ↓ QGC creates MANUAL_CONTROL message
  UDP packet → 192.168.168.101:14550
    ↓ Network transmission
  mavlink_bridge_node UDP socket receive
    ↓ Parse MANUAL_CONTROL payload
  Normalize joystick values (-1000..1000)
    ↓ Convert to PWM range (1100..1900 µs)
  Create Twist message
    ↓ Publish to /motor_data
  guidance_node subscribes
    ↓ Update motor control variables
  guidance_node publishes to /motor_data
    ↓ Micro-ROS relay
  ESP32 receives updated motor command
    ↓ Generate PWM on GPIO pins
  Thruster drivers receive PWM
    ↓ Convert to motor drive current
  T200 Thrusters
    ↓ Physical thrust generated
  ROV Moves!


MESSAGE RATES & BANDWIDTH
═══════════════════════════════════════════════════════════════════════════

Outgoing (QGC upload):
  HEARTBEAT:             1 Hz  ~  30 bytes = ~240 bps
  ATTITUDE:             20 Hz  ~  28 bytes = ~4.5 kbps
  GLOBAL_POSITION_INT:  20 Hz  ~  28 bytes = ~4.5 kbps
  SYS_STATUS:           20 Hz  ~  31 bytes = ~5.0 kbps
                                          ─────────────
                        TOTAL:                ~14 kbps

Incoming (QGC download):
  MANUAL_CONTROL:       ~20 Hz ~  10 bytes = ~1.6 kbps
  COMMAND_LONG:         sporadic           = <1 kbps
                                          ─────────────
                        TOTAL:               ~2-3 kbps

Total Bandwidth: ~16-17 kbps (well within LAN capacity)


NETWORK TOPOLOGY
═══════════════════════════════════════════════════════════════════════════

Option 1: SAME LAN (Recommended for testing)
┌─────────────────────────────────────────────┐
│  Local Network (192.168.168.0/24)           │
│                                              │
│  ┌─────────────┐    Ethernet    ┌─────────┐│
│  │ Your PC     ├─────────────────┤ Router  ││
│  │ .100        │                 │         ││
│  └─────────────┘                 └────┬────┘│
│                                        │    │
│                          ┌─────────────┘    │
│                          │                  │
│                  ┌───────┴────────┐         │
│                  │ UP Board       │         │
│                  │ .101           │         │
│                  │ (WiFi or Eth)  │         │
│                  └────────────────┘         │
│                                              │
└─────────────────────────────────────────────┘

Option 2: SSH TUNNEL (For internet connectivity)
┌──────────────────────────────────────────────────────────┐
│                   INTERNET                                │
│                                                            │
│  ┌──────────────┐                  ┌──────────────┐      │
│  │  Your PC     │                  │  UP Board    │      │
│  │ w/ QGC       │  SSH tunnel      │ rov_ws       │      │
│  └──────────────┘  (encrypted)     └──────────────┘      │
│                                                            │
└──────────────────────────────────────────────────────────┘

Option 3: MOBILE HOTSPOT (For portable operation)
┌──────────────────────────────────────────────────────────┐
│              Mobile Hotspot WiFi                           │
│                  (2.4 GHz or 5 GHz)                       │
│                                                            │
│  ┌──────────────┐              ┌──────────────┐          │
│  │  Your PC     │              │  UP Board    │          │
│  │ WiFi Client  ├──────────────┤ WiFi Client  │          │
│  │  .100        │              │  .101        │          │
│  └──────────────┘              └──────────────┘          │
│                                                            │
└──────────────────────────────────────────────────────────┘


KEY STATISTICS
═══════════════════════════════════════════════════════════════════════════

Code Size:
  mavlink_bridge_node.py:        380 lines
  Total package:                  ~500 lines (including tests)
  Documentation:                  ~35 KB (4 guides + READMEs)

Performance:
  Telemetry Update Rate:          20 Hz (50 ms)
  HEARTBEAT Rate:                 1 Hz (1000 ms)
  Motor Command Latency:          <100 ms (typical LAN)
  UDP Bandwidth:                  ~16 kbps (out of 1000 Mbps available)

Compatibility:
  QGC Version:                    All recent versions (3.5+)
  ROS2 Version:                   Foxy (Ubuntu 20.04)
  Network:                        Ethernet or WiFi (UDP)
  Platform:                       Linux (UP Board), Windows/Mac/Linux (PC with QGC)

═════════════════════════════════════════════════════════════════════════════

                        ✅ INTEGRATION COMPLETE ✅

EOF
