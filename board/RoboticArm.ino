/*
 * ZRO_MEK 6-DOF Anthropomorphic Arm with Spherical Wrist + Gripper
 * Based on classical Denavit-Hartenberg convention
 */

#include <Servo.h>
#include <math.h>

// Arduino.h already defines DEG_TO_RAD, RAD_TO_DEG, and HALF_PI

// ========== ROBOT CONFIGURATION ==========
// Complete DH Parameter Table - Configure for YOUR specific robot
// Based on classical Denavit-Hartenberg convention
//
// | Link |  a_i (mm) | α_i (rad)  | d_i (mm) | θ_i (variable) |
// |------|-----------|------------|----------|----------------|
// |  1   |    a1     |    α1      |    d1    | θ₁ (J1)        |
// |  2   |    a2     |    α2      |    d2    | θ₂ (J2)        |
// |  3   |    a3     |    α3      |    d3    | θ₃ (J3)        |
// |  4   |    a4     |    α4      |    d4    | θ₄ (J4)        |
// |  5   |    a5     |    α5      |    d5    | θ₅ (J5)        |
// |  6   |    a6     |    α6      |    d6    | θ₆ (J6)        |

// *** CONFIGURE YOUR ROBOT'S DH PARAMETERS HERE ***
// Link lengths (mm)
constexpr float DH_a[6] = {30, 70, 70, 0, 0, 0};   
// Link offsets (mm)
constexpr float DH_d[6] = {0, 0, 0, 100, 0, 50};  


// Link twists (rad)          
constexpr float DH_alpha[6] = {HALF_PI, 0, HALF_PI, -HALF_PI, HALF_PI, 0};          


// Servo configuration - 7 servos total (6 DOF + gripper)
constexpr uint8_t PINS[7] = {3, 5, 6, 9, 10, 11, 12};  // Servo pins
constexpr float SERVO_MIN_ANGLES[7] = {0, 0, 0, 0, 0, 0, 0};
constexpr float SERVO_MAX_ANGLES[7] = {180, 180, 180, 180, 180, 180, 180};
constexpr float SERVO_NEUTRALS[7] = {90, 90, 90, 90, 90, 90, 90};

// Motion control
constexpr float MOVE_SPEED = 100.0;  // degrees per second  
constexpr float MAIN_LOOP_DELAY = 20; // 50Hz update rate for smooth motion

// ========== GLOBAL VARIABLES ==========
Servo servos[7];
float angles[7];         // Current joint angles (0-6: joints, gripper)
float target_angles[7];  // Target joint angles
String cmd = "";         // Command buffer
bool is_moving = false;  // Movement state
bool debug_enabled = false;  // Debug logging flag

// ========== UTILITY FUNCTIONS ==========
void debug(const char* msg) {
    if (debug_enabled) {
        Serial.print(F("DBG: "));
        Serial.println(msg);
    }
}

void debugPos(float x, float y, float z) {
    if (debug_enabled) {
        Serial.print(F("DBG: Target pos ("));
        Serial.print(x); Serial.print(F(","));
        Serial.print(y); Serial.print(F(","));
        Serial.print(z); Serial.println(F(")"));
    }
}

void setJoint(int j, float deg) {
    if (j >= 0 && j < 7) {
        float orig_deg = deg;
        deg = constrain(deg, SERVO_MIN_ANGLES[j], SERVO_MAX_ANGLES[j]);
        if (orig_deg != deg && debug_enabled) {
            Serial.print(F("DBG: Joint limit J"));
            Serial.println(j+1);
        }
        angles[j] = deg;
        servos[j].write(deg);
    }
}

void setTargetJoint(int j, float deg) {
    if (j >= 0 && j < 7) {
        target_angles[j] = constrain(deg, SERVO_MIN_ANGLES[j], SERVO_MAX_ANGLES[j]);
    }
}

void startSmoothMove() {
    debug("Motion start");
    is_moving = true;
}

// ========== CONFIGURABLE DH KINEMATICS ==========
// Forward kinematics using configured DH parameters
void forwardKin6DOF(float* pos, float* rot) {
    // Convert servo angles to DH joint angles (radians)
    float theta[6];
    for (int i = 0; i < 6; i++) {
        theta[i] = (angles[i] - SERVO_NEUTRALS[i]) * DEG_TO_RAD;
    }
    
    // Compute transformation matrices using DH parameters
    // T_i^{i-1} = Rot_z(theta_i) * Trans_z(d_i) * Trans_x(a_i) * Rot_x(alpha_i)
    
    // Pre-compute trigonometric values
    float c[6], s[6], ca[6], sa[6];
    for (int i = 0; i < 6; i++) {
        c[i] = cos(theta[i]);
        s[i] = sin(theta[i]);
        ca[i] = cos(DH_alpha[i]);
        sa[i] = sin(DH_alpha[i]);
    }
    
    // Build cumulative transformation T_6^0 = T_1^0 * T_2^1 * ... * T_6^5
    // Using efficient matrix multiplication for position and orientation
    
    // Initialize with identity
    float T[4][4] = {{1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,0,1}};
    
    // Apply each DH transformation
    for (int i = 0; i < 6; i++) {
        // DH transformation matrix for joint i
        float Ti[4][4] = {
            {c[i], -s[i]*ca[i],  s[i]*sa[i], DH_a[i]*c[i]},
            {s[i],  c[i]*ca[i], -c[i]*sa[i], DH_a[i]*s[i]},
            {0,     sa[i],       ca[i],      DH_d[i]     },
            {0,     0,           0,          1           }
        };
        
        // T = T * Ti (matrix multiplication)
        float T_new[4][4];
        for (int r = 0; r < 4; r++) {
            for (int col = 0; col < 4; col++) {
                T_new[r][col] = 0;
                for (int k = 0; k < 4; k++) {
                    T_new[r][col] += T[r][k] * Ti[k][col];
                }
            }
        }
        
        // Copy result back
        for (int r = 0; r < 4; r++) {
            for (int col = 0; col < 4; col++) {
                T[r][col] = T_new[r][col];
            }
        }
    }
    
    // Extract position from transformation matrix
    pos[0] = T[0][3];  // x
    pos[1] = T[1][3];  // y
    pos[2] = T[2][3];  // z
    
    // Extract approach vector (3rd column of rotation matrix)
    rot[0] = T[0][2];  // ax
    rot[1] = T[1][2];  // ay
    rot[2] = T[2][2];  // az
}

// Inverse kinematics for 6-DOF robot using configurable DH parameters
// Implements analytical solution based on kinematic decoupling principle from doc/ik.md
bool inverseKin6DOF(float x, float y, float z, float ax, float ay, float az, float* result_angles) {
    // Step 1: Compute wrist center position (decouple position and orientation)
    // p_W = p_e - d6 * approach_vector (from doc/ik.md)
    float px_w = x - DH_d[5]*ax;
    float py_w = y - DH_d[5]*ay;
    float pz_w = z - DH_d[5]*az;
    
    // Step 2: Solve for first 3 joints using configured DH parameters
    
    // Joint 1 (base rotation): θ₁ = Atan2(p_Wy, p_Wx)
    result_angles[0] = SERVO_NEUTRALS[0] + atan2(py_w, px_w) * RAD_TO_DEG;
    
    // For joints 2&3, we need to solve the 2R planar problem
    // This depends on your specific arm geometry defined in DH_a[] and DH_d[]
    
    // Generic 2R solution - adapt based on your DH configuration:
    float rho = sqrt(px_w*px_w + py_w*py_w);
    
    // If you have an anthropomorphic arm (a2 non-zero):
    if (DH_a[1] > 0.1) {  // Has upper arm link
        // Use 2-link arm solution with a2 and effective forearm length  
        // For anthropomorphic arm: effective forearm combines a3 and d4 geometrically
        float effective_forearm = sqrt(DH_a[2]*DH_a[2] + DH_d[3]*DH_d[3]);  // sqrt(a3² + d4²)
        float c3 = (rho*rho + pz_w*pz_w - DH_a[1]*DH_a[1] - effective_forearm*effective_forearm) / (2*DH_a[1]*effective_forearm);
        
        if (c3 > 1.0 || c3 < -1.0) {
            debug("IK unreachable - outside workspace");
            return false;
        }
        
        float s3_pos = sqrt(1.0 - c3*c3);  // Elbow-up solution
        result_angles[2] = SERVO_NEUTRALS[2] + atan2(s3_pos, c3) * RAD_TO_DEG;
        
        float k1 = DH_a[1] + effective_forearm*c3;
        float k2 = effective_forearm*s3_pos;
        result_angles[1] = SERVO_NEUTRALS[1] + atan2(k1*pz_w - k2*rho, k1*rho + k2*pz_w) * RAD_TO_DEG;
        
    } else {
        // Alternative arm configuration - implement based on your specific DH parameters
        result_angles[1] = SERVO_NEUTRALS[1];  // Default
        result_angles[2] = SERVO_NEUTRALS[2];  // Default
    }
    
    // Step 3: Solve spherical wrist (J4-J6) using ZYZ Euler angles
    // Compute R₃⁰ transformation matrix from first 3 joints
    float q1_rad = (result_angles[0] - SERVO_NEUTRALS[0]) * DEG_TO_RAD;
    float q2_rad = (result_angles[1] - SERVO_NEUTRALS[1]) * DEG_TO_RAD;
    float q3_rad = (result_angles[2] - SERVO_NEUTRALS[2]) * DEG_TO_RAD;
    
    // Build R₃⁰ dynamically using actual DH parameters (first 3 joints)
    // Compute T₃⁰ using same DH transformation as forward kinematics
    float T30[4][4] = {{1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,0,1}};
    
    // Apply first 3 DH transformations to get T₃⁰
    for (int i = 0; i < 3; i++) {
        float theta_i = (result_angles[i] - SERVO_NEUTRALS[i]) * DEG_TO_RAD;
        float c_i = cos(theta_i), s_i = sin(theta_i);
        float ca_i = cos(DH_alpha[i]), sa_i = sin(DH_alpha[i]);
        
        // DH transformation matrix for joint i
        float Ti[4][4] = {
            {c_i, -s_i*ca_i,  s_i*sa_i, DH_a[i]*c_i},
            {s_i,  c_i*ca_i, -c_i*sa_i, DH_a[i]*s_i},
            {0,    sa_i,      ca_i,     DH_d[i]     },
            {0,    0,         0,        1           }
        };
        
        // T30 = T30 * Ti
        float T_new[4][4];
        for (int r = 0; r < 4; r++) {
            for (int col = 0; col < 4; col++) {
                T_new[r][col] = 0;
                for (int k = 0; k < 4; k++) {
                    T_new[r][col] += T30[r][k] * Ti[k][col];
                }
            }
        }
        
        // Copy result back
        for (int r = 0; r < 4; r++) {
            for (int col = 0; col < 4; col++) {
                T30[r][col] = T_new[r][col];
            }
        }
    }
    
    // Extract R₃⁰ from T₃⁰ (top-left 3x3 block)
    float R30[3][3] = {
        {T30[0][0], T30[0][1], T30[0][2]},
        {T30[1][0], T30[1][1], T30[1][2]},
        {T30[2][0], T30[2][1], T30[2][2]}
    };
    
    // Transform approach vector to frame 3
    float ax3 = R30[0][0]*ax + R30[0][1]*ay + R30[0][2]*az;
    float ay3 = R30[1][0]*ax + R30[1][1]*ay + R30[1][2]*az;
    float az3 = R30[2][0]*ax + R30[2][1]*ay + R30[2][2]*az;
    
    // Extract spherical wrist angles (ZYZ Euler angles from doc/ik.md)
    float r_wrist = sqrt(ax3*ax3 + ay3*ay3);
    
    if (r_wrist < 1e-6) {
        // Singular configuration - wrist aligned
        result_angles[3] = SERVO_NEUTRALS[3];  // θ₄ = 0
        result_angles[4] = SERVO_NEUTRALS[4] + atan2(r_wrist, az3) * RAD_TO_DEG;  // θ₅
        result_angles[5] = SERVO_NEUTRALS[5];  // θ₆ = 0
    } else {
        // Non-singular: extract ZYZ Euler angles
        result_angles[3] = SERVO_NEUTRALS[3] + atan2(ay3, ax3) * RAD_TO_DEG;       // θ₄
        result_angles[4] = SERVO_NEUTRALS[4] + atan2(r_wrist, az3) * RAD_TO_DEG;   // θ₅
        result_angles[5] = SERVO_NEUTRALS[5];  // θ₆ = 0 (simplified - full solution needs n,s vectors)
    }
    
    // Constrain all angles to servo limits
    for (int i = 0; i < 6; i++) {
        result_angles[i] = constrain(result_angles[i], SERVO_MIN_ANGLES[i], SERVO_MAX_ANGLES[i]);
    }
    
    return true;
}

void updateSmoothMotion() {
    if (!is_moving) return;
    
    bool all_reached = true;
    static unsigned long last_debug = 0;
    bool show_debug = debug_enabled && (millis() - last_debug > 500);
    
    for (int i = 0; i < 7; i++) {
        float diff = target_angles[i] - angles[i];
        
        if (abs(diff) > 0.1) {
            all_reached = false;
            
            float step = MOVE_SPEED * (MAIN_LOOP_DELAY / 1000.0);
            if (abs(diff) < step) {
                angles[i] = target_angles[i];
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
        Serial.print(F("DBG: Moving J["));
        for (int i = 0; i < 7; i++) {
            Serial.print(angles[i]);
            if (i < 6) Serial.print(F(","));
        }
        Serial.println(F("]"));
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

void getWorkspaceLimits(float* limits) {
    // Returns [x_min, x_max, y_min, y_max, z_min, z_max] based on configured DH parameters
    
    // Calculate maximum reach from configured DH parameters
    float reach_max = 0;
    for (int i = 0; i < 6; i++) {
        reach_max += sqrt(DH_a[i]*DH_a[i] + DH_d[i]*DH_d[i]);  // Combined reach contribution
    }
    
    // Conservative workspace estimation
    limits[0] = -reach_max;  // x_min
    limits[1] = reach_max;   // x_max
    limits[2] = -reach_max;  // y_min  
    limits[3] = reach_max;   // y_max
    limits[4] = -reach_max/2;  // z_min (conservative)
    limits[5] = reach_max;     // z_max
}

// ========== COMMAND PROCESSING ==========
void printState() {
    float pos[3], rot[3];
    forwardKin6DOF(pos, rot);
    
    float limits[6];
    getWorkspaceLimits(limits);
    
    Serial.println(F("OK {"));
    Serial.print(F("  \"joints\": ["));
    for (int i = 0; i < 6; i++) {
        Serial.print(angles[i]);
        if (i < 5) Serial.print(F(", "));
    }
    Serial.println(F("],"));
    Serial.print(F("  \"gripper\": ")); Serial.print(angles[6]); Serial.println(F(","));
    Serial.print(F("  \"pos\": ["));
    Serial.print(pos[0]); Serial.print(F(", "));
    Serial.print(pos[1]); Serial.print(F(", "));
    Serial.print(pos[2]); Serial.println(F("],"));
    Serial.print(F("  \"approach\": ["));
    Serial.print(rot[0]); Serial.print(F(", "));
    Serial.print(rot[1]); Serial.print(F(", "));
    Serial.print(rot[2]); Serial.println(F("],"));
    Serial.print(F("  \"workspace\": {"));
    Serial.print(F("\"x\": [")); Serial.print(limits[0]); Serial.print(F(", ")); Serial.print(limits[1]); Serial.print(F("], "));
    Serial.print(F("\"y\": [")); Serial.print(limits[2]); Serial.print(F(", ")); Serial.print(limits[3]); Serial.print(F("], "));
    Serial.print(F("\"z\": [")); Serial.print(limits[4]); Serial.print(F(", ")); Serial.print(limits[5]); Serial.println(F("]}"));
    Serial.println(F("}"));
}

void processCmd() {
    cmd.trim();
    cmd.toUpperCase();
    
    if (is_moving && cmd != "HELP" && cmd != "STATE" && cmd != "STOP" && !cmd.startsWith("DEBUG")) {
        debug("Command blocked - moving");
        Serial.println(F("ERR BUSY - Wait for current movement to finish"));
        return;
    }
    
    if (cmd == "HELP") {
        Serial.println(F("OK Commands: HELP, STATE, MOVEJ, MOVE, JOG, GRIP, HOME, TEST1, TEST2, TEST3, STOP, DEBUG"));
        Serial.println(F("Examples:"));
        Serial.println(F("  MOVEJ J2 45"));
        Serial.println(F("  MOVE X 150 Y 0 Z 100 AX 1 AY 0 AZ 0"));
        Serial.println(F("  JOG J1 15"));
        Serial.println(F("  GRIP 60"));
        Serial.println(F("  TEST1  // Extreme right reach"));
        Serial.println(F("  TEST2  // High overhead pose"));
        Serial.println(F("  TEST3  // Complex wrist orientation"));
        
    } else if (cmd == "STATE") {
        printState();
        
    } else if (cmd == "HOME") {
        debug("Homing all joints");
        for (int i = 0; i < 7; i++) {
            setTargetJoint(i, SERVO_NEUTRALS[i]);
        }
        startSmoothMove();
        Serial.println(F("OK Moving home"));
        
    } else if (cmd == "TEST1") {
        // Extreme right reach - tests base rotation, shoulder extension, elbow bend
        debug("Moving to TEST1 - extreme right reach");
        setTargetJoint(0, 20);   // Base rotated far right
        setTargetJoint(1, 45);   // Shoulder pitched down
        setTargetJoint(2, 135);  // Elbow bent up
        setTargetJoint(3, 160);  // Wrist roll
        setTargetJoint(4, 45);   // Wrist pitch
        setTargetJoint(5, 135);  // Wrist yaw
        setTargetJoint(6, 45);   // Gripper partially closed
        startSmoothMove();
        Serial.println(F("OK Moving to TEST1 - extreme right reach"));
        
    } else if (cmd == "TEST2") {
        // High overhead pose - tests vertical reach, wrist orientation
        debug("Moving to TEST2 - high overhead pose");
        setTargetJoint(0, 90);   // Base centered
        setTargetJoint(1, 135);  // Shoulder pitched up high
        setTargetJoint(2, 45);   // Elbow extended
        setTargetJoint(3, 45);   // Wrist roll rotated
        setTargetJoint(4, 135);  // Wrist pitched down
        setTargetJoint(5, 60);   // Wrist yaw angled
        setTargetJoint(6, 120);  // Gripper mostly open
        startSmoothMove();
        Serial.println(F("OK Moving to TEST2 - high overhead pose"));
        
    } else if (cmd == "TEST3") {
        // Complex wrist orientation - tests all joints in unusual configuration
        debug("Moving to TEST3 - complex wrist orientation");
        setTargetJoint(0, 160);  // Base rotated far left
        setTargetJoint(1, 120);  // Shoulder angled up
        setTargetJoint(2, 60);   // Elbow partially bent
        setTargetJoint(3, 30);   // Wrist roll extreme
        setTargetJoint(4, 160);  // Wrist pitch extreme
        setTargetJoint(5, 20);   // Wrist yaw extreme
        setTargetJoint(6, 10);   // Gripper nearly closed
        startSmoothMove();
        Serial.println(F("OK Moving to TEST3 - complex wrist orientation"));
        
    } else if (cmd.startsWith("MOVEJ ")) {
        String params = cmd.substring(6);
        int spaceIdx = params.indexOf(' ');
        if (spaceIdx == -1) {
            Serial.println(F("ERR Missing parameters"));
        } else {
            String joint = params.substring(0, spaceIdx);
            params = params.substring(spaceIdx + 1);
            
            if (joint.startsWith("J")) {
                int j = joint.substring(1).toInt() - 1;
                if (j >= 0 && j < 6) {
                    float angle = getFloat(params);
                    float limited_angle = constrain(angle, SERVO_MIN_ANGLES[j], SERVO_MAX_ANGLES[j]);
                    if (debug_enabled) {
                        Serial.print(F("DBG: MovJ target J"));
                        Serial.println(j+1);
                    }
                    setTargetJoint(j, limited_angle);
                    startSmoothMove();
                    
                    Serial.print(F("OK "));
                    if (limited_angle != angle) {
                        Serial.print(F("J"));
                        Serial.print(j+1);
                        Serial.print(F(" limit reached at "));
                        Serial.println(limited_angle);
                    } else {
                        Serial.print(F("Moving J"));
                        Serial.print(j+1);
                        Serial.print(F(" to "));
                        Serial.println(angle);
                    }
                } else {
                    Serial.println(F("ERR Invalid joint"));
                }
            } else {
                Serial.println(F("ERR Invalid joint format"));
            }
        }
        
    } else if (cmd.startsWith("MOVE ")) {
        String params = cmd.substring(5);
        float x = 0, y = 0, z = 0, ax = 0, ay = 0, az = 1;  // Default approach vector
        
        while (params.length() > 0) {
            int spaceIdx = params.indexOf(' ');
            String token;
            
            if (spaceIdx == -1) {
                token = params;
                params = "";
            } else {
                token = params.substring(0, spaceIdx);
                params = params.substring(spaceIdx + 1);
            }
            
            if (token == "X") x = getFloat(params);
            else if (token == "Y") y = getFloat(params);
            else if (token == "Z") z = getFloat(params);
            else if (token == "AX") ax = getFloat(params);
            else if (token == "AY") ay = getFloat(params);
            else if (token == "AZ") az = getFloat(params);
        }
        
        debugPos(x, y, z);
        float result_angles[6];
        if (inverseKin6DOF(x, y, z, ax, ay, az, result_angles)) {
            debug("IK success");
            for (int i = 0; i < 6; i++) setTargetJoint(i, result_angles[i]);
            startSmoothMove();
            Serial.println(F("OK Moving to pose"));
        } else {
            Serial.println(F("ERR UNREACHABLE"));
        }
        
    } else if (cmd.startsWith("JOG ")) {
        String params = cmd.substring(4);
        int spaceIdx = params.indexOf(' ');
        if (spaceIdx == -1) {
            Serial.println(F("ERR Missing parameters"));
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
                        if (debug_enabled) {
                            Serial.print(F("DBG: Limit hit J"));
                            Serial.println(j+1);
                        }
                        Serial.print(F("OK J"));
                        Serial.print(j+1);
                        Serial.print(F(" limit reached at "));
                        Serial.println(limited_angle);
                    } else {
                        if (debug_enabled) {
                            Serial.print(F("DBG: Jog J"));
                            Serial.println(j+1);
                        }
                        Serial.print(F("OK Jogging J"));
                        Serial.print(j+1);
                        Serial.print(F(" to "));
                        Serial.println(limited_angle);
                    }
                } else {
                    Serial.println(F("ERR Invalid joint"));
                }
            } else {
                Serial.println(F("ERR Invalid joint format"));
            }
        }
        
    } else if (cmd.startsWith("GRIP ")) {
        float percent = cmd.substring(5).toFloat();
        if (debug_enabled) {
            Serial.print(F("DBG: Grip to "));
            Serial.println(percent);
        }
        setTargetJoint(6, percent);  // Gripper is servo 7 (index 6)
        startSmoothMove();
        Serial.println(F("OK Moving gripper"));
        
    } else if (cmd == "STOP") {
        debug("Emergency stop");
        is_moving = false;
        Serial.println(F("OK Movement stopped"));
        
    } else if (cmd == "DEBUG ON") {
        debug_enabled = true;
        Serial.println(F("OK Debug enabled"));
        
    } else if (cmd == "DEBUG OFF") {
        debug_enabled = false;
        Serial.println(F("OK Debug disabled"));
        
    } else if (cmd == "DEBUG") {
        Serial.println(debug_enabled ? F("OK Debug ON") : F("OK Debug OFF"));
        
    } else {
        Serial.println(F("ERR Unknown command"));
    }
}

// ========== ARDUINO SETUP/LOOP ==========
void setup() {
    Serial.begin(115200);
    Serial.println(F("ZRO_MEK 6-DOF Anthropomorphic Arm with Spherical Wrist - Ready"));
    
    for (int i = 0; i < 7; i++) {
        servos[i].attach(PINS[i]);
        angles[i] = SERVO_NEUTRALS[i];
        target_angles[i] = SERVO_NEUTRALS[i];
        servos[i].write(SERVO_NEUTRALS[i]);
        delay(100);
    }
    
    Serial.println(F("Type HELP for commands"));
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