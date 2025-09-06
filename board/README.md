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
| `DEBUG ON/OFF` | Enable/disable debug logging | `DEBUG ON` |
| `DEBUG` | Show debug status | `DEBUG` |

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
const float MOVE_SPEED = 50.0;  // Movement speed (degrees/second)
const float MAIN_LOOP_DELAY = 20; // Control loop rate (ms) = 50Hz
```

## Joint Limits

| Joint | Min | Max | Description |
|-------|-----|-----|-------------|
| J1 (Base) | -160° | +160° | Base rotation |
| J2 (Shoulder) | -10° | +180° | Shoulder pitch |
| J3 (Elbow) | -10° | +180° | Elbow pitch |
| J4 (Wrist Pitch) | -90° | +90° | Wrist up/down |
| J5 (Wrist Yaw) | -180° | +180° | Wrist rotation |
| J6 (Gripper) | 0% | 100% | Gripper opening |

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

**Debug output shows:**

- Target positions and IK calculations
- Joint movements and limit enforcement  
- Real-time position/orientation during motion
- Current joint angles every 0.5 seconds

## Motion Control

- **Speed:** 50 degrees/second movement speed
- **Update Rate:** 50Hz control loop (20ms)
- **Precision:** 0.1 degree positioning tolerance
- **Safety:** Command blocking during movement
- **Smooth Motion:** Interpolated movement between positions

## Troubleshooting

- **Servo jitter:** Check power supply capacity
- **"UNREACHABLE" errors:** Position outside workspace - try closer positions  
- **"ERR BUSY" errors:** Wait for movement to finish or use `STOP`
- **Inaccurate positioning:** Enable debug mode to monitor joint angles
