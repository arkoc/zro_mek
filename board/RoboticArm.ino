/*
 * ZRO_MEK 6-DOF Robotic Arm Controller - Lightweight Version for Arduino Uno
 * Reduced memory footprint for simulators
 */

#include <Servo.h>

// ========== CONFIGURATION ==========
constexpr float PLATFORM_HEIGHT = 100;  // Platform height above ground (mm)
constexpr float L1 = 10, L2 = 120, L3 = 120, L4 = 40, L5 = 60;  // Link lengths (mm)
constexpr uint8_t PINS[6] = {3, 5, 6, 9, 10, 11};  // Servo pins
constexpr float SERVO_MIN_ANGLES[6] = {0, 0, 0, 0, 0, 0};
constexpr float SERVO_MAX_ANGLES[6] = {270, 180, 180, 180, 270, 180};
constexpr float SERVO_NEUTRALS[6] = {135, 90, 90, 90, 135, 90};  // Based on LIMITS arrays
constexpr float MOVE_SPEED = 100.0;  // degrees per second  
constexpr float MAIN_LOOP_DELAY = 20; // 50Hz update rate for smooth motion

// ========== GLOBAL VARIABLES ==========
Servo servos[6];
float angles[6];  // Current joint angles (set to servo neutrals in setup)
float target_angles[6];  // Target joint angles (set to servo neutrals in setup)
String cmd = "";  // Command buffer
bool is_moving = false;  // Movement state
bool debug_enabled = false;  // Debug logging flag

// ========== SERVO CONFIGURATION ==========
inline float servoToKinematics(int joint, float servo_angle) {
    // Convert servo angle to kinematics angle (degrees from neutral)
    return servo_angle - SERVO_NEUTRALS[joint];
}

inline float kinematicsToServo(int joint, float kinematics_angle) {
    // Convert kinematics angle (degrees from neutral) to servo angle
    return SERVO_NEUTRALS[joint] + kinematics_angle;
}

// ========== UTILITY FUNCTIONS ==========
void debug(const char* msg) {
    if (debug_enabled) {
        Serial.print("DBG: ");
        Serial.println(msg);
    }
}

void debugFloat(const char* msg, float val) {
    if (debug_enabled) {
        Serial.print("DBG: ");
        Serial.print(msg);
        Serial.println(val);
    }
}

void debugPos(float x, float y, float z) {
    if (debug_enabled) {
        Serial.print("DBG: Target pos (");
        Serial.print(x); Serial.print(",");
        Serial.print(y); Serial.print(",");
        Serial.print(z); Serial.println(")");
    }
}
void setJoint(int j, float deg) {
    if (j >= 0 && j < 6) {
        float orig_deg = deg;
        deg = constrain(deg, SERVO_MIN_ANGLES[j], SERVO_MAX_ANGLES[j]);
        if (orig_deg != deg) {
            debugFloat("Joint limit J", (float)(j+1));
        }
        angles[j] = deg;
        servos[j].write(deg);
    }
}

void setTargetJoint(int j, float deg) {
    if (j >= 0 && j < 6) {
        target_angles[j] = constrain(deg, SERVO_MIN_ANGLES[j], SERVO_MAX_ANGLES[j]);
    }
}

void startSmoothMove() {
    debug("Motion start");
    is_moving = true;
}

void updateSmoothMotion() {
    if (!is_moving) return;
    
    bool all_reached = true;
    static unsigned long last_debug = 0;
    bool show_debug = debug_enabled && (millis() - last_debug > 500);  // Debug every 0.5s during motion
    
    for (int i = 0; i < 6; i++) {
        float diff = target_angles[i] - angles[i];
        
        if (abs(diff) > 0.1) {  // 0.1 degree tolerance for better precision
            all_reached = false;
            
            // Calculate step based on speed and actual loop delay
            float step = MOVE_SPEED * (MAIN_LOOP_DELAY / 1000.0);  // Convert ms to seconds
            if (abs(diff) < step) {
                angles[i] = target_angles[i];  // Snap to exact target
            } else {
                angles[i] += (diff > 0) ? step : -step;
            }
            
            servos[i].write(angles[i]);
        }
    }
    
    if (all_reached) {
        debug("Motion complete");
        is_moving = false;
    } else if (show_debug) {
        // Show current joint angles during movement (avoid expensive forward kinematics)
        Serial.print("DBG: Moving J[");
        for (int i = 0; i < 6; i++) {
            Serial.print(angles[i]);
            if (i < 5) Serial.print(",");
        }
        Serial.println("]");
        last_debug = millis();
    }
}

float getFloat(String& str) {
    int idx = str.indexOf(' ');
    if (idx > 0) {
        String val = str.substring(0, idx);
        str = str.substring(idx + 1);
        return val.toFloat();
    }
    float val = str.toFloat();
    str = "";
    return val;
}

// ========== BASIC KINEMATICS ==========
// Forward kinematics for full 6-DOF
void forwardKin6DOF(float* pos, float* rot) {
    float j1 = servoToKinematics(0, angles[0]) * DEG_TO_RAD;
    float j2 = servoToKinematics(1, angles[1]) * DEG_TO_RAD;
    float j3 = servoToKinematics(2, angles[2]) * DEG_TO_RAD;
    float j4 = servoToKinematics(3, angles[3]) * DEG_TO_RAD;
    float j5 = servoToKinematics(4, angles[4]) * DEG_TO_RAD;
    
    // Position calculation (same as 3DOF but includes wrist)
    float s1 = sin(j1), c1 = cos(j1);
    float s2 = sin(j2), c2 = cos(j2);
    float s23 = sin(j2 + j3), c23 = cos(j2 + j3);
    float s4 = sin(j4), c4 = cos(j4);
    float s5 = sin(j5), c5 = cos(j5);
    
    // End-effector position (wrist center, not gripper tip)
    float r = L2 * c2 + L3 * c23 + L4 * c23 * c4;  // Remove L5 - gripper doesn't affect pose
    pos[0] = r * s1;  // x
    pos[1] = PLATFORM_HEIGHT + L1 + L2 * s2 + L3 * s23 + L4 * s23 * c4;  // y (includes platform, no L5)
    pos[2] = r * c1;  // z
    
    // End-effector orientation (simplified for direct wrist control)
    rot[0] = 0;  // Roll - this arm has no roll capability around end-effector axis
    rot[1] = j4 * RAD_TO_DEG;  // Pitch - direct wrist pitch
    rot[2] = j5 * RAD_TO_DEG;  // Yaw - direct wrist yaw
}

// 6-DOF Inverse Kinematics
bool inverseKin6DOF(float x, float y, float z, float rx, float ry, float rz, float* result_angles) {
    // For simplicity, solve position first, then orientation
    
    // Step 1: Solve for base rotation (J1) - servo-aware conversion
    float base_angle_deg = atan2(x, z) * RAD_TO_DEG;
    result_angles[0] = kinematicsToServo(0, base_angle_deg);
    
    // Step 2: Calculate 2D problem in arm plane
    float r = sqrt(x*x + z*z) - L4;  // Distance from base to wrist (no L5)
    float h = (y - PLATFORM_HEIGHT) - L1;  // Height from base (subtract platform height)
    float d = sqrt(r*r + h*h);  // Distance to wrist center
    
    // Check reachability
    if (d > L2 + L3 || d < abs(L2 - L3) || d < 0.1) {
        debug("IK unreachable");
        return false;  // Prevent division by zero
    }
    
    // Step 3: Solve shoulder (J2) and elbow (J3) - servo-aware conversion
    float cos_a = (L2*L2 + d*d - L3*L3) / (2*L2*d);
    cos_a = constrain(cos_a, -1.0, 1.0);  // Prevent acos domain error
    float a = acos(cos_a);
    float b = atan2(h, r);
    float shoulder_deg = (a + b) * RAD_TO_DEG;
    result_angles[1] = kinematicsToServo(1, shoulder_deg);
    
    float cos_c = (L2*L2 + L3*L3 - d*d) / (2*L2*L3);
    cos_c = constrain(cos_c, -1.0, 1.0);  // Prevent acos domain error
    float c = acos(cos_c);
    float elbow_deg = (3.14159 - c) * RAD_TO_DEG;
    result_angles[2] = kinematicsToServo(2, elbow_deg);
    
    // Step 4: Solve wrist orientation (J4, J5)
    
    // Wrist control - direct degree mapping
    result_angles[3] = kinematicsToServo(3, ry);  // Wrist pitch
    result_angles[4] = kinematicsToServo(4, rz);  // Wrist yaw
    
    if (debug_enabled) {
        Serial.print("DBG: Wrist calc - ry:");
        Serial.print(ry);
        Serial.print(" rz:");
        Serial.print(rz);
        Serial.print(" J4:");
        Serial.print(result_angles[3]);
        Serial.print(" J5:");
        Serial.println(result_angles[4]);
    }
    
    // Constrain all angles to servo limits
    for (int i = 0; i < 5; i++) {
        float original = result_angles[i];
        result_angles[i] = constrain(result_angles[i], SERVO_MIN_ANGLES[i], SERVO_MAX_ANGLES[i]);
        if (debug_enabled && original != result_angles[i]) {
            Serial.print("DBG: J");
            Serial.print(i+1);
            Serial.print(" clamped: ");
            Serial.print(original);
            Serial.print(" -> ");
            Serial.println(result_angles[i]);
        }
    }
    
    return true;
}

void getWorkspaceLimits(float* limits) {
    // Returns [x_min, x_max, y_min, y_max, z_min, z_max]
    float reach_max = L2 + L3 + L4;  // Maximum reach (no L5 - gripper doesn't affect workspace)
    
    limits[0] = -reach_max;  // x_min
    limits[1] = reach_max;   // x_max
    limits[2] = PLATFORM_HEIGHT + L1 - (L2 + L3);  // y_min (lowest point including platform)
    limits[3] = PLATFORM_HEIGHT + L1 + L2 + L3;    // y_max (highest point including platform)
    limits[4] = -reach_max;  // z_min
    limits[5] = reach_max;   // z_max
}

// ========== COMMAND PROCESSING ==========
void printState() {
    float pos[3], rot[3];
    forwardKin6DOF(pos, rot);
    
    float limits[6];
    getWorkspaceLimits(limits);
    
    Serial.println("OK {");
    Serial.print("  \"joints\": [");
    for (int i = 0; i < 6; i++) {
        Serial.print(angles[i]);
        if (i < 5) Serial.print(", ");
    }
    Serial.println("],");
    Serial.print("  \"pos\": [");
    Serial.print(pos[0]); Serial.print(", ");
    Serial.print(pos[1]); Serial.print(", ");
    Serial.print(pos[2]); Serial.println("],");
    Serial.print("  \"rot\": [");
    Serial.print(rot[0]); Serial.print(", ");
    Serial.print(rot[1]); Serial.print(", ");
    Serial.print(rot[2]); Serial.println("],");
    Serial.print("  \"workspace\": {");
    Serial.print("\"x\": ["); Serial.print(limits[0]); Serial.print(", "); Serial.print(limits[1]); Serial.print("], ");
    Serial.print("\"y\": ["); Serial.print(limits[2]); Serial.print(", "); Serial.print(limits[3]); Serial.print("], ");
    Serial.print("\"z\": ["); Serial.print(limits[4]); Serial.print(", "); Serial.print(limits[5]); Serial.println("]}");
    Serial.println("}");
}

void processCmd() {
    cmd.trim();
    cmd.toUpperCase();
    
    // Block movement commands during execution (except status/help/stop commands)
    if (is_moving && cmd != "HELP" && cmd != "STATE" && cmd != "STOP" && !cmd.startsWith("DEBUG")) {
        debug("Command blocked - moving");
        Serial.println("ERR BUSY - Wait for current movement to finish");
        return;
    }
    
    if (cmd == "HELP") {
        Serial.println("OK Commands: HELP, STATE, MOVEJ, MOVEL, JOG, GRIP, HOME, STOP, DEBUG");
        Serial.println("Examples:");
        Serial.println("  MOVEJ J2 45");
        Serial.println("  MOVEL X 150 Y 200 Z 100 RX 0 RY 0 RZ 0");
        Serial.println("  JOG J1 15");
        
    } else if (cmd == "STATE") {
        printState();
        
    } else if (cmd == "HOME") {
        debug("Homing all joints");
        for (int i = 0; i < 6; i++) {
            setTargetJoint(i, SERVO_NEUTRALS[i]);  // Move to neutral position
        }
        startSmoothMove();
        Serial.println("OK Moving home");
        
    } else if (cmd.startsWith("MOVEJ ")) {
        String params = cmd.substring(6);
        int spaceIdx = params.indexOf(' ');
        if (spaceIdx == -1) {
            Serial.println("ERR Missing parameters");
        } else {
            String joint = params.substring(0, spaceIdx);
            params = params.substring(spaceIdx + 1);
            
            if (joint.startsWith("J")) {
                int j = joint.substring(1).toInt() - 1;
                if (j >= 0 && j < 6) {
                    float angle = getFloat(params);
                    float limited_angle = constrain(angle, SERVO_MIN_ANGLES[j], SERVO_MAX_ANGLES[j]);
                    debugFloat("MovJ target J", (float)(j+1));
                    setTargetJoint(j, limited_angle);
                    startSmoothMove();
                    
                    if (limited_angle != angle) {
                        Serial.println("OK J" + String(j+1) + " limit reached at " + String(limited_angle));
                    } else {
                        Serial.println("OK Moving J" + String(j+1) + " to " + String(angle));
                    }
                } else {
                    Serial.println("ERR Invalid joint");
                }
            } else {
                Serial.println("ERR Invalid joint format");
            }
        }
        
    } else if (cmd.startsWith("MOVEL ")) {
        String params = cmd.substring(6);
        float x = 0, y = 0, z = 0, rx = 0, ry = 0, rz = 0;
        
        while (params.length() > 0) {
            int spaceIdx = params.indexOf(' ');
            String token;
            
            if (spaceIdx == -1) {
                // Last parameter - no more spaces
                token = params;
                params = "";
            } else {
                token = params.substring(0, spaceIdx);
                params = params.substring(spaceIdx + 1);
            }
            
            if (token == "X") x = getFloat(params);
            else if (token == "Y") y = getFloat(params);
            else if (token == "Z") z = getFloat(params);
            else if (token == "RX") rx = getFloat(params);
            else if (token == "RY") ry = getFloat(params);
            else if (token == "RZ") rz = getFloat(params);
        }
        
        
        debugPos(x, y, z);
        float result_angles[6];
        if (inverseKin6DOF(x, y, z, rx, ry, rz, result_angles)) {
            debug("IK success");
            for (int i = 0; i < 5; i++) setTargetJoint(i, result_angles[i]);  // Set target for first 5 joints
            startSmoothMove();
            Serial.println("OK Moving to pose");
        } else {
            Serial.println("ERR UNREACHABLE");
        }
        
    } else if (cmd.startsWith("JOG ")) {
        String params = cmd.substring(4);
        int spaceIdx = params.indexOf(' ');
        if (spaceIdx == -1) {
            Serial.println("ERR Missing parameters");
        } else {
            String joint = params.substring(0, spaceIdx);
            params = params.substring(spaceIdx + 1);
            
            if (joint.startsWith("J")) {
                int j = joint.substring(1).toInt() - 1;
                if (j >= 0 && j < 6) {
                    float jog_amount = getFloat(params);
                    float current_angle = angles[j];
                    float requested_angle = current_angle + jog_amount;
                    float limited_angle = constrain(requested_angle, SERVO_MIN_ANGLES[j], SERVO_MAX_ANGLES[j]);
                    
                    setTargetJoint(j, limited_angle);
                    startSmoothMove();
                    
                    if (limited_angle != requested_angle) {
                        debugFloat("Limit hit J", (float)(j+1));
                        Serial.println("OK J" + String(j+1) + " limit reached at " + String(limited_angle));
                    } else {
                        debugFloat("Jog J", (float)(j+1));
                        Serial.println("OK Jogging J" + String(j+1) + " to " + String(limited_angle));
                    }
                } else {
                    Serial.println("ERR Invalid joint");
                }
            } else {
                Serial.println("ERR Invalid joint format");
            }
        }
        
    } else if (cmd.startsWith("GRIP ")) {
        float percent = cmd.substring(5).toFloat();
        debugFloat("Grip to ", percent);
        setTargetJoint(5, percent);
        startSmoothMove();
        Serial.println("OK Moving gripper");
        
    } else if (cmd == "STOP") {
        debug("Emergency stop");
        is_moving = false;
        Serial.println("OK Movement stopped");
        
    } else if (cmd == "DEBUG ON") {
        debug_enabled = true;
        Serial.println("OK Debug enabled");
        
    } else if (cmd == "DEBUG OFF") {
        debug_enabled = false;
        Serial.println("OK Debug disabled");
        
    } else if (cmd == "DEBUG") {
        Serial.println(debug_enabled ? "OK Debug ON" : "OK Debug OFF");
        
    } else {
        Serial.println("ERR Unknown command");
    }
}

// ========== ARDUINO SETUP/LOOP ==========
void setup() {
    Serial.begin(115200);
    Serial.println("ZRO_MEK 6-DOF Robotic Arm - Ready");
    
    for (int i = 0; i < 6; i++) {
        servos[i].attach(PINS[i]);
        angles[i] = SERVO_NEUTRALS[i];
        target_angles[i] = SERVO_NEUTRALS[i];
        servos[i].write(SERVO_NEUTRALS[i]);
        delay(100);  // Give servos time to reach position
    }
    
    Serial.println("Type HELP for commands");
}

void loop() {
    // Handle serial commands
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (cmd.length() > 0) {
                processCmd();
                cmd = "";
            }
        } else if (c >= 32) {
            cmd += c;
        }
    }
    
    // Update smooth motion
    updateSmoothMotion();
    
    delay(MAIN_LOOP_DELAY);
}
