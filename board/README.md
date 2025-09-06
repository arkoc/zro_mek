# 6-DOF Robotic Arm Controller (ZRO_MEK)

Simple Arduino Uno controller for a 6-servo robotic arm with basic kinematics and serial commands.

## Hardware Setup

**Arduino Uno + 6 Servo Motors**

| Servo | Joint | Pin | Function |
|-------|-------|-----|----------|
| S1 | Base Yaw | D3 | Base rotation |
| S2 | Shoulder | D5 | Shoulder pitch |
| S3 | Elbow | D6 | Elbow pitch |
| S4 | Wrist Pitch | D9 | Wrist up/down |
| S5 | Wrist Yaw | D10 | Wrist rotation |
| S6 | Gripper | D11 | Open/close |

**Power:** Use external 5-6V power supply for servos (Arduino USB power is insufficient)

## Configuration

Edit these values in the code to match your robot:

```cpp
constexpr float L1 = 0;    // Base to shoulder height (mm)
constexpr float L2 = 120;  // Shoulder to elbow length (mm)
constexpr float L3 = 120;  // Elbow to wrist length (mm)
constexpr float L4 = 40;   // Wrist adapter length (mm)
constexpr float L5 = 60;   // Gripper extension length (mm)
```

## Upload and Test

1. **Upload** `RoboticArmLite.ino` to Arduino Uno
2. **Open Serial Monitor** at 115200 baud
3. **Type** `HELP` to see available commands

## Commands

| Command | Description | Example |
|---------|-------------|---------|
| `HELP` | Show all commands | `HELP` |
| `GET STATE` | Show joint angles and position | `GET STATE` |
| `HOME` | Move all joints to zero | `HOME` |
| `MOVEJ <angles>` | Move to joint angles (degrees) | `MOVEJ 0 45 -30 0 0 50` |
| `MOVEL X <x> Y <y> Z <z>` | Move to XYZ position (mm) | `MOVEL X 150 Y 120 Z 80` |
| `JOG J<n> <degrees>` | Jog single joint | `JOG J2 15` |
| `GRIP <percent>` | Set gripper position | `GRIP 75` |
| `CALIB START` | Enter calibration mode | `CALIB START` |
| `CALIB SETZERO` | Set current position as zero | `CALIB SETZERO` |

## Quick Start

```
HOME                           // Move to home position
GET STATE                      // See current status
MOVEJ 0 45 -30 0 0 50         // Move joints
MOVEL X 150 Y 120 Z 80        // Move to position
JOG J2 15                     // Fine adjust shoulder
GRIP 75                       // Open gripper 75%
```

## Coordinate System

- **Origin:** Center of base rotation
- **X:** Right (positive = right side)
- **Y:** Up (positive = upward)
- **Z:** Forward (positive = forward from base)

## Joint Limits

The code enforces these soft limits to prevent damage:

| Joint | Min | Max | Description |
|-------|-----|-----|-------------|
| J1 (Base) | -160° | +160° | Base rotation |
| J2 (Shoulder) | -10° | +180° | Shoulder pitch |
| J3 (Elbow) | -10° | +180° | Elbow pitch |
| J4 (Wrist Pitch) | -90° | +90° | Wrist up/down |
| J5 (Wrist Yaw) | -180° | +180° | Wrist rotation |
| J6 (Gripper) | 0% | 100% | Gripper opening |

## Troubleshooting

**Servo jitter:** Check power supply capacity  
**Inaccurate movement:** Verify link lengths L1-L5 match your hardware  
**Serial issues:** Ensure 115200 baud rate  
**"UNREACHABLE" errors:** Target position outside robot workspace  

## Memory Optimized

This lite version is specifically designed for Arduino Uno's limited memory:
- Single file with minimal code
- Basic 3DOF kinematics (full 6DOF positioning still works)
- Direct servo control without motion planning
- Essential commands only

Perfect for learning, prototyping, and simulator testing!

---
*Upload `RoboticArmLite.ino` and start controlling your robotic arm with simple serial commands.*
