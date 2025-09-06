/*
 * 6-DOF Robotic Arm Controller - Lightweight Version for Arduino Uno
 * Reduced memory footprint for simulators
 */

#include <Servo.h>

// ========== CONFIGURATION ==========
constexpr float PLATFORM_HEIGHT = 100;  // Platform height above ground (mm)
constexpr float L1 = 10, L2 = 120, L3 = 120, L4 = 40, L5 = 60;  // Link lengths (mm)
constexpr uint8_t PINS[6] = {3, 5, 6, 9, 10, 11};  // Servo pins
constexpr float LIMITS_MIN[6] = {-160, -10, -10, -90, -180, 0};
constexpr float LIMITS_MAX[6] = {160, 180, 180, 90, 180, 100};
const float MOVE_SPEED = 50.0;  // degrees per second
const float MAIN_LOOP_DELAY = 20; // 50Hz update rate for smooth motion

// ========== GLOBAL VARIABLES ==========
Servo servos[6];
float angles[6] = {0, 0, 0, 0, 0, 0};  // Current joint angles
float target_angles[6] = {0, 0, 0, 0, 0, 0};  // Target joint angles
String cmd = "";  // Command buffer
bool is_moving = false;  // Movement state
bool debug_enabled = false;  // Debug logging flag

// ========== UTILITY FUNCTIONS ==========
int angleToPulse(float deg) {
    return constrain(1500 + deg * 11.11, 500, 2500);  // Simple linear mapping
}

void setJoint(int j, float deg) {
    if (j >= 0 && j < 6) {
        deg = constrain(deg, LIMITS_MIN[j], LIMITS_MAX[j]);
        angles[j] = deg;
        servos[j].writeMicroseconds(angleToPulse(deg));
    }
}

void setTargetJoint(int j, float deg) {
    if (j >= 0 && j < 6) {
        target_angles[j] = constrain(deg, LIMITS_MIN[j], LIMITS_MAX[j]);
    }
}

void startSmoothMove() {
    is_moving = true;
}

void updateSmoothMotion() {
    if (!is_moving) return;
    
    bool all_reached = true;
    
    for (int i = 0; i < 6; i++) {
        float diff = target_angles[i] - angles[i];
        
        if (abs(diff) > 1.0) {  // 1 degree tolerance
            all_reached = false;
            
            // Calculate step based on speed and actual loop delay
            float step = MOVE_SPEED * (MAIN_LOOP_DELAY / 1000.0);  // Convert ms to seconds
            if (abs(diff) < step) {
                angles[i] = target_angles[i];
            } else {
                angles[i] += (diff > 0) ? step : -step;
            }
            
            servos[i].writeMicroseconds(angleToPulse(angles[i]));
        }
    }
    
    if (all_reached) {
        is_moving = false;
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
    float j1 = angles[0] * 0.01745;  // DEG_TO_RAD
    float j2 = angles[1] * 0.01745;
    float j3 = angles[2] * 0.01745;
    float j4 = angles[3] * 0.01745;
    float j5 = angles[4] * 0.01745;
    
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
    
    // End-effector orientation (proper Euler angles)
    // For a typical 6-DOF arm: Roll=0 (no roll axis), Pitch=arm+wrist, Yaw=base+wrist_yaw
    rot[0] = 0;  // Roll - this arm has no roll capability around end-effector axis
    rot[1] = (j2 + j3 + j4) * 57.296;  // Pitch - cumulative arm and wrist pitch
    rot[2] = (j1 + j5) * 57.296;  // Yaw - base rotation + wrist yaw
}

// 6-DOF Inverse Kinematics
bool inverseKin6DOF(float x, float y, float z, float rx, float ry, float rz, float* result_angles) {
    // For simplicity, solve position first, then orientation
    
    // Step 1: Solve for base rotation (J1)
    result_angles[0] = atan2(x, z) * 57.296;  // RAD_TO_DEG
    
    // Step 2: Calculate 2D problem in arm plane
    float r = sqrt(x*x + z*z) - L4;  // Distance from base to wrist (no L5)
    float h = (y - PLATFORM_HEIGHT) - L1;  // Height from base (subtract platform height)
    float d = sqrt(r*r + h*h);  // Distance to wrist center
    
    // Check reachability
    if (d > L2 + L3 || d < abs(L2 - L3) || d < 0.1) {
        return false;  // Prevent division by zero
    }
    
    // Step 3: Solve shoulder (J2) and elbow (J3)
    float cos_a = (L2*L2 + d*d - L3*L3) / (2*L2*d);
    cos_a = constrain(cos_a, -1.0, 1.0);  // Prevent acos domain error
    float a = acos(cos_a);
    float b = atan2(h, r);
    result_angles[1] = (a + b) * 57.296;  // Shoulder
    
    float cos_c = (L2*L2 + L3*L3 - d*d) / (2*L2*L3);
    cos_c = constrain(cos_c, -1.0, 1.0);  // Prevent acos domain error
    float c = acos(cos_c);
    result_angles[2] = (3.14159 - c) * 57.296;  // Elbow
    
    // Step 4: Solve wrist orientation (J4, J5)
    // Current arm pitch from shoulder and elbow
    float arm_pitch = result_angles[1] + result_angles[2];
    
    // Desired end-effector orientation
    result_angles[3] = ry - arm_pitch;  // Wrist pitch to achieve desired RY
    result_angles[4] = rz - result_angles[0];  // Wrist yaw compensated for base rotation
    
    // Constrain all angles to limits
    for (int i = 0; i < 5; i++) {
        result_angles[i] = constrain(result_angles[i], LIMITS_MIN[i], LIMITS_MAX[i]);
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
    if (is_moving && cmd != "HELP" && cmd != "GET_STATE" && cmd != "STOP") {
        Serial.println("ERR BUSY - Wait for current movement to finish");
        return;
    }
    
    if (cmd == "HELP") {
        Serial.println("OK Commands: HELP, GET_STATE, MOVEJ, MOVEL, JOG, GRIP, HOME, STOP");
        Serial.println("Examples:");
        Serial.println("  MOVEJ J2 45");
        Serial.println("  MOVEL X 150 Y 200 Z 100 RX 0 RY 0 RZ 0");
        Serial.println("  JOG J1 15");
        
    } else if (cmd == "GET_STATE") {
        printState();
        
    } else if (cmd == "HOME") {
        for (int i = 0; i < 6; i++) setTargetJoint(i, 0);
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
                    setTargetJoint(j, angle);
                    startSmoothMove();
                    Serial.println("OK Moving J" + String(j+1) + " to " + String(angle));
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
        
        
        float result_angles[6];
        if (inverseKin6DOF(x, y, z, rx, ry, rz, result_angles)) {
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
                    float limited_angle = constrain(requested_angle, LIMITS_MIN[j], LIMITS_MAX[j]);
                    
                    setTargetJoint(j, limited_angle);
                    startSmoothMove();
                    
                    if (limited_angle != requested_angle) {
                        Serial.println("OK J" + String(j+1) + " limit reached at " + String(limited_angle));
                    } else {
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
        setTargetJoint(5, percent);
        startSmoothMove();
        Serial.println("OK Moving gripper");
        
    } else if (cmd == "STOP") {
        is_moving = false;
        Serial.println("OK Movement stopped");
        
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
        angles[i] = 0;
        target_angles[i] = 0;
        servos[i].writeMicroseconds(angleToPulse(0));
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
