# Aurora Vehicle Driver - Home Assignment

This project implements a ROS 2 Python vehicle controller for the Aurora home assignment, using CAN communication over a virtual CAN interface (`vcan0`).

The controller performs the requested mission flow:
- release handbrake
- send ignition command
- wait for engine RPM feedback
- shift to DRIVE
- accelerate for a configurable duration
- log vehicle status
- apply braking
- engage handbrake and shut the engine off

## Package Structure
- `aurora_driver/` – ROS 2 Python nodes
- `launch/` – launch file for running the mock vehicle and controller together
- `canbridge/` – minimal local compatibility layer created because the original assignment package was missing
- `setup.py`, `setup.cfg`, `package.xml` – ROS 2 package configuration

## Prerequisites
- Ubuntu 22.04
- ROS 2 Humble
- `python-can`

Install `python-can` if needed:
```bash
pip3 install python-can
```

## 1. Virtual CAN Setup
Before running the nodes, ensure the `vcan0` interface is active:
```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

## 2. Build
From the workspace root:
```bash
cd ~/ros2_ws
colcon build --packages-select aurora_driver
source install/setup.bash
```

## 3. Launch
```bash
# Default run
ros2 launch aurora_driver aurora_driver.launch.py

# Custom parameters
ros2 launch aurora_driver aurora_driver.launch.py drive_duration_sec:=3.0 drive_throttle:=-300
```

## Implemented Features
- Startup retry loop until valid engine RPM and released handbrake are detected
- 20 Hz heartbeat publishing over CAN
- Configurable throttle and drive duration through ROS 2 parameters
- Parsing of:
  - vehicle speed
  - engine RPM
  - battery SOC
- Controlled braking and shutdown sequence
- Timer cleanup and CAN bus shutdown during exit

## Mission Sequence
1. Release handbrake
2. Send ignition command
3. Wait until RPM feedback indicates the engine is running
4. Shift to DRIVE
5. Start throttle shortly after shifting to DRIVE
6. Log speed and RPM
7. Apply braking
8. Engage handbrake and send engine-off command

A sample successful run is provided in `simulation_log.txt`.