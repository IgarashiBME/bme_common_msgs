# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

ROS2 message-only package (`bme_common_msgs`) defining custom message types for u-blox GNSS receivers based on the UBX protocol specification (u-blox F9 HPG 1.50).

## Build Commands

```bash
# Build
cd ~/ros2_ws && colcon build --packages-select bme_common_msgs

# Source and test
source ~/ros2_ws/install/setup.bash
ros2 run bme_common_msgs test_publisher.py

# Verify message definition
ros2 interface show bme_common_msgs/msg/PVT
```

## Message Definitions

All messages include `std_msgs/Header header` (except AutoLog which uses `builtin_interfaces/Time stamp`).

### Receiver-specific messages (u-blox UBX protocol)

| Message | UBX Equivalent | Description |
|---------|---------------|-------------|
| PVT.msg | UBX-NAV-PVT (0x01 0x07) | Navigation Position Velocity Time |
| RELPOSNED.msg | UBX-NAV-RELPOSNED (0x01 0x3C) | Relative positioning in NED frame |
| HPPOSLLH.msg | UBX-NAV-HPPOSLLH (0x01 0x14) | High precision geodetic position |
| UTMHP.msg | Custom | UTM coordinates with high-precision GNSS |

### Receiver-independent messages

| Message | Description |
|---------|-------------|
| GnssSolution.msg | Unified GNSS solution (position, UTM, heading, accuracy) independent of receiver hardware |
| AutoLog.msg | Autonomous navigation logging |
| MavModes.msg | MAVLink mode information |

## ROS2 Jazzy Naming Rules (Critical)

Build will fail if these rules are violated:
- **Message file names**: PascalCase only (`PVT.msg`, not `Nav_PVT.msg`)
- **Field names**: snake_case only (`rel_pos_n`, not `relPosN`)
- **Time fields**: Use `builtin_interfaces/Time` or `std_msgs/Header`

## UBX Type Mapping

| UBX | ROS2 | Notes |
|-----|------|-------|
| U1/I1 | uint8/int8 | |
| U2/I2 | uint16/int16 | |
| U4/I4 | uint32/int32 | |
| X1/X2/X4 | uint8/uint16/uint32 | Bitfields - document bits in comments |
| R4/R8 | float32/float64 | |

## Adding New Messages

See `doc/adding_new_message.md` for detailed instructions. Key steps:
1. Create `.msg` file in `msg/` with PascalCase name
2. Add to `CMakeLists.txt` in `rosidl_generate_interfaces()`
3. Update `scripts/test_publisher.py` if needed

## Reference

UBX protocol PDF in `references/` directory.
