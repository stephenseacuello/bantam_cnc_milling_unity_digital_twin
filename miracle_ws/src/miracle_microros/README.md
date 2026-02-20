# miracle_microros

Firmware configurations and transport setup for micro-ROS MCU bridges (ESP32, STM32) in the MIRACLE manufacturing digital twin system.

This package provides:

- **Sensor acquisition firmware** for ESP32 (vibration, current) and STM32 (temperature, acoustic emission, coolant flow)
- **Transport layer configuration** for WiFi/UDP and Serial/UART bridges to the micro-ROS agent
- **Pin mapping and calibration** parameters for all analog sensor channels
- **micro-ROS node configuration** including publishers, subscribers, timers, and memory allocation

## Prerequisites

### Host Machine

1. **ROS 2 Humble** (or later) with micro-ROS packages:
   ```bash
   sudo apt install ros-humble-micro-ros-setup ros-humble-micro-ros-agent
   ```

2. **PlatformIO CLI** (for building and flashing firmware):
   ```bash
   pip install platformio
   ```

3. **Docker** (optional, for running the micro-ROS agent in a container):
   ```bash
   sudo apt install docker.io
   ```

### Hardware

- **ESP32 DevKit V1** - Vibration and current sensor acquisition
- **STM32 Nucleo F446RE** - Temperature, acoustic, and coolant flow acquisition
- USB cables for flashing and serial transport
- Factory WiFi network for ESP32 UDP transport

## Directory Structure

```
miracle_microros/
├── CMakeLists.txt              # ROS 2 build configuration
├── package.xml                 # ROS 2 package manifest
├── README.md                   # This file
├── config/
│   ├── transport_params.yaml   # Transport layer settings (UDP, serial, QoS)
│   ├── sensor_mapping.yaml     # Pin-to-topic mapping and calibration
│   └── node_config.yaml        # micro-ROS node entity definitions
└── firmware/
    ├── esp32/
    │   ├── platformio.ini      # PlatformIO build config for ESP32
    │   └── src/
    │       └── main.cpp        # ESP32 sensor bridge firmware
    └── stm32/
        ├── platformio.ini      # PlatformIO build config for STM32
        └── src/
            └── main.cpp        # STM32 sensor bridge firmware
```

## Building the ROS 2 Package

The ROS 2 package installs configuration files and firmware source references into the workspace share directory. No C++ compilation happens on the host side -- the MCU firmware is built separately via PlatformIO.

```bash
cd ~/miracle_ws
colcon build --packages-select miracle_microros
source install/setup.bash
```

## Flashing Firmware

### ESP32

1. Connect the ESP32 DevKit to your computer via USB.

2. Configure WiFi credentials and agent IP. Edit `firmware/esp32/platformio.ini` or create a `platformio_override.ini` file:
   ```ini
   [env:esp32dev]
   build_flags =
       ${env:esp32dev.build_flags}
       -D MIRACLE_WIFI_SSID=\"YourFactorySSID\"
       -D MIRACLE_WIFI_PASSWORD=\"YourPassword\"
       -D MIRACLE_AGENT_IP=\"192.168.1.100\"
       -D MIRACLE_MACHINE_ID=\"cnc_002\"
   ```

3. Build and flash:
   ```bash
   cd miracle_ws/src/miracle_microros/firmware/esp32
   pio run --target upload
   ```

4. Monitor serial output:
   ```bash
   pio device monitor --baud 115200
   ```

### STM32

1. Connect the Nucleo F446RE to your computer via the ST-Link USB connector.

2. (Optional) Adjust machine ID or publish rates in `firmware/stm32/platformio.ini`.

3. Build and flash:
   ```bash
   cd miracle_ws/src/miracle_microros/firmware/stm32
   pio run --target upload
   ```

4. Monitor serial output:
   ```bash
   pio device monitor --baud 115200
   ```

## Network Configuration

### ESP32 (WiFi / UDP Transport)

The ESP32 connects to the micro-ROS agent over WiFi using UDP on port 8888. Ensure the following:

- The factory WiFi network is reachable from the ESP32.
- The micro-ROS agent host machine is on the same subnet (default: 192.168.1.1).
- UDP port 8888 is open on the agent host firewall.
- All nodes use ROS domain ID 42 (configured in `config/transport_params.yaml`).

### STM32 (Serial / UART Transport)

The STM32 connects to the micro-ROS agent over a wired serial link at 921600 baud via the ST-Link USB interface. The default device path on Linux is `/dev/ttyUSB0` (or `/dev/ttyACM0` for some boards).

To grant serial port access without root:
```bash
sudo usermod -aG dialout $USER
# Log out and back in for the group change to take effect
```

## Running the micro-ROS Agent

### Option 1: Native Agent

Start the agent for UDP transport (ESP32):
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 \
    --ros-args --remap __ns:=/miracle
```

Start the agent for serial transport (STM32):
```bash
ros2 run micro_ros_agent micro_ros_agent serial \
    --dev /dev/ttyUSB0 -b 921600 \
    --ros-args --remap __ns:=/miracle
```

### Option 2: Docker Container

Run the micro-ROS agent in a Docker container (useful for CI/testing):

```bash
# UDP agent for ESP32
docker run -it --rm --net=host \
    microros/micro-ros-agent:humble \
    udp4 --port 8888 --ros-args --remap __ns:=/miracle

# Serial agent for STM32
docker run -it --rm --net=host \
    --device=/dev/ttyUSB0 \
    microros/micro-ros-agent:humble \
    serial --dev /dev/ttyUSB0 -b 921600 --ros-args --remap __ns:=/miracle
```

## Testing

### Verify Agent Connection

After flashing the firmware and starting the agent, verify the MCU nodes are visible:

```bash
# List all nodes (should show /miracle/miracle_sensor_esp32 and/or
# /miracle/miracle_sensor_stm32)
ros2 node list

# List topics
ros2 topic list | grep miracle

# Echo vibration data from ESP32
ros2 topic echo /miracle/cnc_001/raw_sensor/vibration

# Echo temperature from STM32
ros2 topic echo /miracle/cnc_001/raw_sensor/temperature

# Check heartbeat
ros2 topic echo /miracle/cnc_001/heartbeat

# Verify publish rates
ros2 topic hz /miracle/cnc_001/raw_sensor/vibration
ros2 topic hz /miracle/cnc_001/raw_sensor/current
```

### Send Commands

Test the E-stop and calibration commands:

```bash
# Trigger emergency stop
ros2 topic pub --once /miracle/cnc_001/command std_msgs/msg/String "data: 'ESTOP'"

# Clear E-stop and resume
ros2 topic pub --once /miracle/cnc_001/command std_msgs/msg/String "data: 'RESET'"

# Request calibration
ros2 topic pub --once /miracle/cnc_001/command std_msgs/msg/String "data: 'CALIBRATE'"
```

## Topic Reference

| Topic | Message Type | Source | Rate |
|-------|-------------|--------|------|
| `/miracle/{machine_id}/raw_sensor/vibration` | `sensor_msgs/Float32MultiArray` | ESP32 | 1000 Hz |
| `/miracle/{machine_id}/raw_sensor/current` | `std_msgs/Float32` | ESP32 | 500 Hz |
| `/miracle/{machine_id}/raw_sensor/temperature` | `std_msgs/Float32` | STM32 | 10 Hz |
| `/miracle/{machine_id}/raw_sensor/acoustic` | `std_msgs/Float32` | STM32 | 1000 Hz |
| `/miracle/{machine_id}/raw_sensor/coolant_flow` | `std_msgs/Float32` | STM32 | 10 Hz |
| `/miracle/{machine_id}/heartbeat` | `std_msgs/UInt32` | ESP32 | 1 Hz |
| `/miracle/{machine_id}/heartbeat_stm32` | `std_msgs/UInt32` | STM32 | 1 Hz |
| `/miracle/{machine_id}/command` | `std_msgs/String` | Host | On demand |

## LED Status Indicators

Both MCU firmware implementations use an onboard LED to indicate system state:

| Pattern | Meaning |
|---------|---------|
| Solid ON | Normal operation, agent connected |
| Slow blink (500 ms) | Connecting to WiFi / agent |
| Fast blink (100 ms) | Error state |
| OFF | E-stop active |

## Troubleshooting

- **ESP32 cannot connect to WiFi**: Verify SSID and password in `platformio.ini`. Ensure the ESP32 is within range. Check that the WiFi network supports 2.4 GHz (ESP32 does not support 5 GHz).
- **Agent not found**: Ensure the micro-ROS agent is running and reachable. For UDP, verify the agent IP and port match the firmware configuration. For serial, check the device path and baud rate.
- **No topics visible**: Confirm the ROS domain ID matches across all nodes (default: 42). Run `export ROS_DOMAIN_ID=42` before using ROS 2 CLI tools.
- **Sensor readings are wrong**: Check the calibration offsets and scale factors in `config/sensor_mapping.yaml` and the firmware source. Verify sensor wiring and ADC pin assignments.
- **Watchdog reset**: The ESP32 firmware uses a 10-second hardware watchdog. If the main loop stalls (e.g., blocked on agent reconnection), the MCU will restart automatically.
