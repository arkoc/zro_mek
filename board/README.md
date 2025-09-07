# 6-DOF Anthropomorphic Robotic Arm with Spherical Wrist (ZRO_MEK)

Arduino controller for a 6-DOF anthropomorphic robotic arm with spherical wrist + gripper based on classical Denavit-Hartenberg convention.

## Hardware Setup

**Robot Configuration:** 6-DOF Anthropomorphic Arm with Spherical Wrist + Gripper

| Servo | Joint | Pin | Function | DH Parameters |
|-------|-------|-----|----------|---------------|
| S1 | Base (J1) | D3 | Base rotation | θ₁, d₁=0, a₁=0, α₁=π/2 |
| S2 | Shoulder (J2) | D5 | Shoulder pitch | θ₂, d₂=0, a₂=150mm, α₂=0 |
| S3 | Elbow (J3) | D6 | Elbow pitch | θ₃, d₃=0, a₃=0, α₃=π/2 |
| S4 | Wrist Roll (J4) | D9 | Wrist rotation | θ₄, d₄=100mm, a₄=0, α₄=-π/2 |
| S5 | Wrist Pitch (J5) | D10 | Wrist up/down | θ₅, d₅=0, a₅=0, α₅=π/2 |
| S6 | Wrist Yaw (J6) | D11 | End-effector rotation | θ₆, d₆=50mm, a₆=0, α₆=0 |
| S7 | Gripper | D12 | Open/close | Independent |

**Power:** Use external 6V power supply for servos (7 servos total).

## Robot Specifications

- **Type:** 6-DOF Anthropomorphic Arm with Spherical Wrist
- **Kinematics:** Classical Denavit-Hartenberg convention
- **Upper Arm Length (a₂):** 150mm
- **Forearm Length (d₄):** 100mm  
- **Wrist Extension (d₆):** 50mm
- **Maximum Reach:** 300mm
- **Workspace:** Spherical with anthropomorphic constraints

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
| `MOVE X <x> Y <y> Z <z> AX <ax> AY <ay> AZ <az>` | Move to position with approach vector | `MOVE X 200 Y 0 Z 150 AX 0 AY 0 AZ 1` |
| `JOG J<n> <degrees>` | Jog joint incrementally | `JOG J2 15` |
| `GRIP <angle>` | Set gripper position | `GRIP 60` |
| `TEST1` | Extreme right reach pose | `TEST1` |
| `TEST2` | High overhead pose | `TEST2` |
| `TEST3` | Complex wrist orientation | `TEST3` |
| `STOP` | Stop current movement | `STOP` |
| `DEBUG ON/OFF` | Enable/disable debug logging | `DEBUG ON` |
| `DEBUG` | Show debug status | `DEBUG` |

## Examples

```
HOME                                        // Move to neutral position (90°)
STATE                                      // See current robot state  
MOVEJ J1 45                               // Move base to 45°
MOVEJ J2 60                               // Move shoulder to 60°
MOVE X 200 Y 0 Z 150 AX 0 AY 0 AZ 1      // Move to position with downward approach
JOG J3 -15                                // Fine adjust elbow -15°
GRIP 45                                   // Close gripper to 45°
TEST1                                     // Extreme right reach (all joints)
TEST2                                     // High overhead pose (all joints)
TEST3                                     // Complex wrist orientation (all joints)
STOP                                      // Emergency stop
```

## Configuration

**Configure your robot's complete DH parameter table in the code:**

```cpp
// *** CONFIGURE YOUR ROBOT'S DH PARAMETERS HERE ***
constexpr float DH_a[6] = {0, 150, 0, 0, 0, 0};           // Link lengths (mm)
constexpr float DH_alpha[6] = {HALF_PI, 0, HALF_PI, -HALF_PI, HALF_PI, 0};  // Link twists (rad)  
constexpr float DH_d[6] = {0, 0, 0, 100, 0, 50};          // Link offsets (mm)

// Motion control
constexpr float MOVE_SPEED = 100.0;  // degrees per second  
constexpr float MAIN_LOOP_DELAY = 20; // Control loop rate (ms) = 50Hz
```

**DH Parameter Explanation:**
- **DH_a[]**: Link lengths between joint axes (mm)
- **DH_alpha[]**: Link twists around x-axis (radians)  
- **DH_d[]**: Link offsets along z-axis (mm)
- **DH_theta[]**: Joint angles (automatically computed from servo positions)

## Joint Limits

| Joint | Min | Max | Neutral | Description |
|-------|-----|-----|---------|-------------|
| J1 (Base) | 0° | 180° | 90° | Base rotation |
| J2 (Shoulder) | 0° | 180° | 90° | Shoulder pitch |
| J3 (Elbow) | 0° | 180° | 90° | Elbow pitch |
| J4 (Wrist Roll) | 0° | 180° | 90° | Wrist rotation |
| J5 (Wrist Pitch) | 0° | 180° | 90° | Wrist up/down |
| J6 (Wrist Yaw) | 0° | 180° | 90° | End-effector rotation |
| Gripper | 0° | 180° | 90° | Gripper opening |

## Coordinate System

**Position:**
- **X:** Right (positive = right side from robot's perspective)
- **Y:** Forward (positive = forward from base)
- **Z:** Up (positive = upward from base level)

**Approach Vector (AX, AY, AZ):**
- **AX:** X-component of end-effector approach direction
- **AY:** Y-component of end-effector approach direction  
- **AZ:** Z-component of end-effector approach direction
- Default: `AX=0, AY=0, AZ=1` (pointing down)

## Kinematics

**Forward Kinematics:**
- Based on classical Denavit-Hartenberg convention
- Computes end-effector position and approach vector from joint angles
- Uses analytical DH transformation matrices

**Inverse Kinematics:**
- Analytical solution using kinematic decoupling
- Arm positioning (J1-J3) solved first for wrist center
- Spherical wrist orientation (J4-J6) solved using ZYZ Euler angles
- Elbow-up configuration preferred

## State Output

```json
{
  "joints": [90.0, 90.0, 90.0, 90.0, 90.0, 90.0],
  "gripper": 90.0,
  "pos": [200.0, 0.0, 150.0],
  "approach": [0.0, 0.0, 1.0],
  "workspace": {"x": [-300, 300], "y": [-300, 300], "z": [-100, 250]}
}
```

## Debug Logging

Enable detailed debugging to monitor robot operation:

```
DEBUG ON                    // Enable debug logging
MOVE X 200 Y 0 Z 150 AX 0 AY 0 AZ 1
DEBUG OFF                   // Disable debug logging
```

**Debug output shows:**
- Target positions and IK calculations
- Joint movements and limit enforcement  
- Real-time joint angles during motion
- Workspace reachability checks

## Motion Control

- **Speed:** 100 degrees/second movement speed
- **Update Rate:** 50Hz control loop (20ms)
- **Precision:** 0.1 degree positioning tolerance
- **Safety:** Command blocking during movement
- **Smooth Motion:** Interpolated movement between positions

## Troubleshooting

- **Servo jitter:** Check power supply capacity (7 servos need adequate current)
- **"UNREACHABLE" errors:** Position outside workspace - try positions within 300mm reach  
- **"ERR BUSY" errors:** Wait for movement to finish or use `STOP`
- **Inaccurate positioning:** Enable debug mode to monitor joint angles and IK solutions
- **Wrist singularities:** Avoid configurations where approach vector aligns with wrist axes

## Technical Details

**Your Configured DH Parameters:**

| Link | a_i (mm) | α_i (rad) | d_i (mm) | θ_i (variable) |
|------|----------|-----------|----------|----------------|
| 1 | DH_a[0] | DH_alpha[0] | DH_d[0] | θ₁ (J1) |
| 2 | DH_a[1] | DH_alpha[1] | DH_d[1] | θ₂ (J2) |
| 3 | DH_a[2] | DH_alpha[2] | DH_d[2] | θ₃ (J3) |
| 4 | DH_a[3] | DH_alpha[3] | DH_d[3] | θ₄ (J4) |
| 5 | DH_a[4] | DH_alpha[4] | DH_d[4] | θ₅ (J5) |
| 6 | DH_a[5] | DH_alpha[5] | DH_d[5] | θ₆ (J6) |

**Example Configuration (modify for your robot):**
- DH_a = {0, 150, 0, 0, 0, 0} mm
- DH_alpha = {π/2, 0, π/2, -π/2, π/2, 0} rad
- DH_d = {0, 0, 0, 100, 0, 50} mm

**Workspace Characteristics:**
- **Type:** Anthropomorphic with spherical wrist dexterity
- **Reachable:** All positions within 300mm radius (with some orientation)
- **Dexterous:** Subset of reachable space with full orientation control
- **Singularities:** Shoulder, elbow, and wrist singularities exist