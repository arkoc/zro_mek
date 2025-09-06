# 6-DOF Robotic Arm Controller (ZRO_MEK)

Arduino Uno controller for a 6-servo robotic arm with kinematics and serial commands.

## Hardware Setup

| Servo | Joint | Pin | Function |
|-------|-------|-----|----------|
| S1 | Base Yaw | D3 | Base rotation |
| S2 | Shoulder | D5 | Shoulder pitch |
| S3 | Elbow | D6 | Elbow pitch |
| S4 | Wrist Pitch | D9 | Wrist up/down |
| S5 | Wrist Yaw | D10 | Wrist rotation |
| S6 | Gripper | D11 | Open/close |

**Power:** Use external 5-6V power supply for servos.

## Upload and Test

1. Upload `RoboticArm.ino` to Arduino Uno
2. Open Serial Monitor at 115200 baud  
3. Type `HELP` to see commands

## Commands

| Command | Description | Example |
|---------|-------------|---------|
| `HELP` | Show all commands | `HELP` |
| `GET_STATE` | Show current robot state | `GET_STATE` |
| `HOME` | Move all joints to zero | `HOME` |
| `MOVEJ J<n> <angle>` | Move joint to angle | `MOVEJ J2 45` |
| `MOVEL X <x> Y <y> Z <z> RX <rx> RY <ry> RZ <rz>` | Move to position | `MOVEL X 150 Y 200 Z 100 RX 0 RY 0 RZ 0` |
| `JOG J<n> <degrees>` | Jog joint incrementally | `JOG J2 15` |
| `GRIP <percent>` | Set gripper position | `GRIP 75` |
| `STOP` | Stop current movement | `STOP` |

## Examples

```
HOME                                    // Move to home position
GET_STATE                              // See current robot state  
MOVEJ J1 90                           // Move base to 90°
MOVEJ J2 45                           // Move shoulder to 45°
MOVEL X 150 Y 200 Z 100 RX 0 RY 0 RZ 0  // Move to position
JOG J2 15                             // Fine adjust shoulder +15°
GRIP 75                               // Open gripper 75%
STOP                                  // Emergency stop
```

## Configuration

Edit these values in the code to match your robot:

```cpp
constexpr float PLATFORM_HEIGHT = 100;  // Platform height (mm)
constexpr float L1 = 10;   // Base to shoulder height (mm)
constexpr float L2 = 120;  // Shoulder to elbow length (mm)  
constexpr float L3 = 120;  // Elbow to wrist length (mm)
constexpr float L4 = 40;   // Wrist adapter length (mm)
constexpr float L5 = 60;   // Gripper extension length (mm)
```

## Coordinate System

**Position:**
- **X:** Right (positive = right side)
- **Y:** Up (positive = upward from ground level)
- **Z:** Forward (positive = forward from base)

**Orientation:**
- **RX:** Roll around X-axis (degrees)
- **RY:** Pitch around Y-axis (degrees)
- **RZ:** Yaw around Z-axis (degrees)

## Troubleshooting

- **Servo jitter:** Check power supply capacity
- **"UNREACHABLE" errors:** Position outside workspace - try closer positions
- **"ERR BUSY" errors:** Wait for movement to finish or use `STOP`