# ROS2 Robot Control System - `sfbot_can`

This repository contains the ROS2 control implementation for a robotic system that communicates with motors via a CAN bus.

The package is designed to work with multi-joint robots and includes various controllers, hardware interfaces, and execution configurations.

---

## Overview

The `sfbot_can` package provides a complete framework for controlling motors connected via a CAN bus using `ros2_control`. It includes the following features:

- Implementation of hardware interfaces for CAN communication (single and multi-CAN bus)
- Motor data processing and management

---

## Key Components

### Hardware Components
- **Motor CAN Driver**: Implements single and multi-CAN interface communication
- **Hardware Interface**: ROS2 hardware interface implementation for the robot

### Controllers
- **Joint Trajectory Controller**: For smooth trajectory execution
- **Forward Position Controller**: For direct position commands
- **Joint State Broadcaster**: For publishing joint states

---

## Executables

- **`test.launch.py`**: Launch file for testing the system

---

## Hardware Support

This system supports CAN-based motors with the following features:

- Position control
- Velocity control
- Combined position-velocity control
- Homing (origin initialization)
- Multi-CAN bus management

---

## License

This package is distributed under the [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0).