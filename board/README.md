# 6-DOF Robotic Arm Controller (ZRO_MEK)

Arduino Uno controller for a 6-servo robotic arm with kinematics and serial commands.

🚀 **Try it online:** [Wokwi Simulator](https://wokwi.com/projects/441360464016898049)

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
| `STATE` | Show current robot state | `STATE` |
| `HOME` | Move all joints to neutral (90°) | `HOME` |
| `MOVEJ J<n> <angle>` | Move joint to angle | `MOVEJ J2 45` |
| `MOVEL X <x> Y <y> Z <z> RX <rx> RY <ry> RZ <rz>` | Move to position | `MOVEL X 150 Y 200 Z 100 RX 0 RY 0 RZ 0` |
| `JOG J<n> <degrees>` | Jog joint incrementally | `JOG J2 15` |
| `GRIP <percent>` | Set gripper position | `GRIP 75` |
| `STOP` | Stop current movement | `STOP` |
| `TEST` | Run 7-step kinematic workspace test | `TEST` |
| `DEBUG ON/OFF` | Enable/disable debug logging | `DEBUG ON` |
| `DEBUG` | Show debug status | `DEBUG` |

## Examples

```
HOME                                    // Move to neutral position (90°)
STATE                                  // See current robot state  
MOVEJ J1 90                           // Move base to 90°
MOVEJ J2 45                           // Move shoulder to 45°
MOVEL X 150 Y 200 Z 100 RX 0 RY 0 RZ 0  // Move to position
JOG J2 15                             // Fine adjust shoulder +15°
GRIP 75                               // Open gripper 75%
TEST                                  // Run 7-step kinematic workspace test
STOP                                  // Emergency stop
```

## Configuration

Edit these values in the code to match your robot:

```cpp
constexpr float PLATFORM_HEIGHT = 50;   // Platform height (mm)
constexpr float L1 = 30;   // Base to shoulder height (mm)
constexpr float L2 = 80;   // Shoulder to elbow length (mm)  
constexpr float L3 = 80;   // Elbow to wrist length (mm)
constexpr float L4 = 10;   // Wrist adapter length (mm)
constexpr float L5 = 0;    // Gripper extension length (mm) - disabled
constexpr float MOVE_SPEED = 100.0;  // Movement speed (degrees/second)
constexpr float MAIN_LOOP_DELAY = 20; // Control loop rate (ms) = 50Hz
```

## Joint Limits

| Joint | Min | Max | Neutral | Description |
|-------|-----|-----|---------|-------------|
| J1 (Base) | 0° | 180° | 90° | Base rotation |
| J2 (Shoulder) | 0° | 180° | 90° | Shoulder pitch |
| J3 (Elbow) | 0° | 180° | 90° | Elbow pitch |
| J4 (Wrist Pitch) | 0° | 180° | 90° | Wrist up/down |
| J5 (Wrist Yaw) | 0° | 180° | 90° | Wrist rotation |
| J6 (Gripper) | 0° | 180° | 90° | Gripper opening |

## Coordinate System

**Position:**
- **X:** Right (positive = right side)
- **Y:** Up (positive = upward from ground level)
- **Z:** Forward (positive = forward from base)

**Orientation:**
- **RX:** Roll around X-axis (degrees)
- **RY:** Pitch around Y-axis (degrees)
- **RZ:** Yaw around Z-axis (degrees)

## Debug Logging

Enable detailed debugging to monitor robot operation:

```
DEBUG ON                    // Enable debug logging
MOVEL X 150 Y 200 Z 100 RX 0 RY 0 RZ 0
DEBUG OFF                   // Disable debug logging
```

## Troubleshooting

- **Servo jitter:** Check power supply capacity
- **"UNREACHABLE" errors:** Position outside workspace - try closer positions  
- **"ERR BUSY" errors:** Wait for movement to finish or use `STOP`
- **Inaccurate positioning:** Enable debug mode to monitor joint angles
- **Testing robot:** Use `TEST` command to run 7-step workspace validation with dimension-based positions covering close/far, high/low, left/right areas
