# Aurora Vehicle Driver - Home Assignment

This ROS 2 package implements a professional driver for the Aurora vehicle, utilizing CAN communication over a virtual CAN interface (`vcan0`). The driver handles the full vehicle lifecycle: automated startup, heartbeat maintenance with throttle control, speed monitoring, and a safe shutdown sequence.

## Package Structure
As per the requirements, the package follows the standard ROS 2 layout:
- `aurora_driver/`: Python source code and node implementation.
- `canbridge/`: CAN protocol constants and message definitions.
- `launch/`: Launch files for simultaneous node execution.
- `setup.py` & `package.xml`: Build and dependency configurations.

## Prerequisites
- **Operating System:** Ubuntu 22.04 (recommended)
- **ROS 2 Version:** Humble or later
- **Python Libraries:** `python-can`

## 1. Virtual CAN Setup
Before running the nodes, ensure the `vcan0` interface is active:
```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

## 2. Installation & Building
To build the package, clone this repository into the src folder of your ROS 2 workspace:
```bash
# Navigate to workspace root
cd ~/ros2_ws

# Build the package
colcon build --packages-select aurora_driver

# Source the environment
source install/setup.bash
```

## 3. Launching the System
```bash
# Basic launch (defaults: 5s duration, -300 throttle)
ros2 launch aurora_driver aurora_driver.launch.py

# Launch with custom mission parameters
ros2 launch aurora_driver aurora_driver.launch.py drive_duration_sec:=3.0 drive_throttle:=-400
```

## Key Features & Professional Implementation
- **Robust Hardware Synchronization:** Implements an adaptive startup retry-loop. The driver monitors vehicle feedback and retries initialization until the simulation is ready, preventing race conditions.
- **Dynamic Mission Control:** Uses ROS 2 parameters for throttle and duration, allowing mission adjustments without code changes.
- **Real-time Telemetry:** Active parsing of Battery SOC (Extended CAN ID `0x18904010`) and vehicle velocity feedback.
- **Safety Fallback:** Integrated exception handling that triggers a heartbeat with **ESTOP active** (Byte 7) upon CAN interface failure.
- **Resource Management:** Ensures clean shutdown by canceling all ROS timers and securing the vehicle state on exit.

## 4. Expected Output & Sequence
The driver follows the automated sequence required by section 5.1 of the protocol:

1. **Startup:** Disengages handbrake and sends ignition command.

2. **Shift:** Waits for engine feedback and shifts gear to DRIVE.

3. **Drive:** Accelerates based on configurable throttle parameters.

4. **Log:** Captures and logs vehicle speed and SOC from CAN messages.

5. **Brake:** Applies brakes for a fixed safety duration.

6. **Shutdown:** Re-engages handbrake and turns the engine off.

Check the simulation_log.txt file in this repository for a sample log of a successful run.