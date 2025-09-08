/*
 * ZRO_MEK 6-DOF Anthropomorphic Arm with Spherical Wrist + Gripper
 */

#include <Servo.h>
#include <math.h>

// ========== ROBOT CONFIGURATION ==========
// Standard Anthropomorphic Arm with Spherical Wrist DH Parameters
// *** CONFIGURE YOUR ROBOT'S VARIABLE DH PARAMETERS HERE ***
// Based on standard Anthropomorphic Arm with Spherical Wrist 
// Only these parameters are configurable - all others are fixed by the kinematic structure
constexpr float DH_a2 = 150;  // hand1
constexpr float DH_a3 = 250; // hand2 + connector_top
constexpr float DH_d6 = 50; // tip + claw (minus actual claws)
constexpr float DH_d1 = 10; // base -> connector_bottom

// Servo configuration - 7 servos total (6 DOF + gripper)
constexpr uint8_t PINS[7] = {3, 5, 6, 9, 10, 11, 12};  // Servo pins
constexpr float SERVO_MIN_ANGLES[7] = {-90, -90, -90, -90, -90, -90, -90};
constexpr float SERVO_MAX_ANGLES[7] = {90, 90, 90, 90, 90, 90, 90};
constexpr float SERVO_NEUTRALS[7] = {0, 0, 0, 0, 0, 0, 0};

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
// Convert from logical angle (-90 to +90) to servo PWM angle (0 to 180)
inline int logicalToServo(float logical_angle) {
    return (int)(logical_angle + 90);  // -90 becomes 0, +90 becomes 180, 0 becomes 90
}

void debug(const char* msg) {
    if (debug_enabled) {
        Serial.print(F("DBG: "));
        Serial.println(msg);
    }
}

void debugPos(float x, float y, float z) {
    if (debug_enabled) {
        Serial.print(F("DBG: Target pos ("));
        Serial.print(x); Serial.print(',');
        Serial.print(y); Serial.print(',');
        Serial.print(z); Serial.println(')');
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
        servos[j].write(logicalToServo(deg));
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


// Build T = [ R(rpy) | p ] for pose (x,y,z, roll,pitch,yaw)
// Intrinsic Z-Y-X convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
// Units: radians
void makeTransformFromRPY(float x, float y, float z,
                          float roll, float pitch, float yaw,
                          float (&T)[4][4]) {

  const float cr = cos(roll),  sr = sin(roll);
  const float cp = cos(pitch), sp = sin(pitch);
  const float cy = cos(yaw),   sy = sin(yaw);

  // Rotation matrix (top-left 3x3)
  T[0][0] = cy*cp;
  T[0][1] = cy*sp*sr - sy*cr;
  T[0][2] = cy*sp*cr + sy*sr;

  T[1][0] = sy*cp;
  T[1][1] = sy*sp*sr + cy*cr;
  T[1][2] = sy*sp*cr - cy*sr;

  T[2][0] = -sp;
  T[2][1] = cp*sr;
  T[2][2] = cp*cr;

  // Translation (last column)
  T[0][3] = x;
  T[1][3] = y;
  T[2][3] = z;

  // Bottom row
  T[3][0] = 0.0; T[3][1] = 0.0; T[3][2] = 0.0; T[3][3] = 1.0;
}

// ========== DH KINEMATICS ==========
void forwardKin6DOF(float* pos, float* rot) {
    if (pos == nullptr || rot == nullptr) return;
    
    // Convert angles to radians
    float q1 = (angles[0] - SERVO_NEUTRALS[0]) * DEG_TO_RAD;
    float q2 = (angles[1] - SERVO_NEUTRALS[1]) * DEG_TO_RAD;
    float q3 = (angles[2] - SERVO_NEUTRALS[2]) * DEG_TO_RAD;
    float q4 = (angles[3] - SERVO_NEUTRALS[3]) * DEG_TO_RAD;
    float q5 = (angles[4] - SERVO_NEUTRALS[4]) * DEG_TO_RAD;
    float q6 = (angles[5] - SERVO_NEUTRALS[5]) * DEG_TO_RAD;
    
    // DH transformation matrices for 6-DOF anthropomorphic arm with spherical wrist
    // Standard DH parameters for this configuration:
    // Joint 1: theta=q1, d=DH_d1, a=0, alpha=90°
    // Joint 2: theta=q2, d=0, a=DH_a2, alpha=0°
    // Joint 3: theta=q3, d=0, a=DH_a3, alpha=0°
    // Joint 4: theta=q4, d=0, a=0, alpha=90°
    // Joint 5: theta=q5, d=0, a=0, alpha=-90°
    // Joint 6: theta=q6, d=DH_d6, a=0, alpha=0°
    
    // Pre-compute trig functions
    float c1 = cos(q1), s1 = sin(q1);
    float c2 = cos(q2), s2 = sin(q2);
    float c3 = cos(q3), s3 = sin(q3);
    float c4 = cos(q4), s4 = sin(q4);
    float c5 = cos(q5), s5 = sin(q5);
    float c6 = cos(q6), s6 = sin(q6);
    
    float c23 = cos(q2 + q3), s23 = sin(q2 + q3);
    
    // Forward kinematics for position (end-effector position)
    pos[0] = c1 * (DH_a2*c2 + DH_a3*c23 + DH_d6*(s23*s4*s5 + c23*c5));
    pos[1] = s1 * (DH_a2*c2 + DH_a3*c23 + DH_d6*(s23*s4*s5 + c23*c5));
    pos[2] = DH_d1 + DH_a2*s2 + DH_a3*s23 + DH_d6*(c23*s4*s5 - s23*c5);
    
    // Forward kinematics for orientation - compute full rotation matrix
    // R = R1*R2*R3*R4*R5*R6 (product of all DH rotations)
    float R11 = c1*(c23*(c4*c5*c6 - s4*s6) - s23*s5*c6) - s1*(s4*c5*c6 + c4*s6);
    float R12 = c1*(c23*(-c4*c5*s6 - s4*c6) + s23*s5*s6) - s1*(-s4*c5*s6 + c4*c6);
    float R13 = c1*(c23*c4*s5 + s23*c5) - s1*s4*s5;
    
    float R21 = s1*(c23*(c4*c5*c6 - s4*s6) - s23*s5*c6) + c1*(s4*c5*c6 + c4*s6);
    float R22 = s1*(c23*(-c4*c5*s6 - s4*c6) + s23*s5*s6) + c1*(-s4*c5*s6 + c4*c6);
    float R23 = s1*(c23*c4*s5 + s23*c5) + c1*s4*s5;
    
    float R31 = -s23*(c4*c5*c6 - s4*s6) - c23*s5*c6;
    float R32 = -s23*(-c4*c5*s6 - s4*c6) + c23*s5*s6;
    float R33 = -s23*c4*s5 + c23*c5;
    
    // Extract RPY angles from rotation matrix (Z-Y-X intrinsic convention)
    // This is the inverse of the rotation sequence used in makeTransformFromRPY
    
    // Extract pitch (rotation about Y-axis)
    float pitch = asin(-R31);
    
    // Check for gimbal lock
    if (abs(cos(pitch)) < 0.001) {
        // Gimbal lock case - set roll to 0 and solve for yaw
        rot[0] = 0; // roll = 0
        rot[1] = pitch * RAD_TO_DEG; // pitch
        rot[2] = atan2(-R12, R22) * RAD_TO_DEG; // yaw
    } else {
        // Normal case
        rot[0] = atan2(R32, R33) * RAD_TO_DEG; // roll
        rot[1] = pitch * RAD_TO_DEG; // pitch  
        rot[2] = atan2(R21, R11) * RAD_TO_DEG; // yaw
    }
}


bool inverseKin6DOF(float x, float y, float z, float roll, float pitch, float yaw, float* result_angles) {
    // Input validation
    if (result_angles == nullptr) {
        debug("IK error: null result array");
        return false;
    }

    float T[4][4];
    makeTransformFromRPY(x, y,z, roll, pitch, yaw, T);

    float p_Wx = x - DH_d6 * T[0][2];
    float p_Wy = y - DH_d6 * T[1][2];
    float p_Wz = z - DH_d6 * T[2][2];
    
    float t1 = atan2(p_Wy, p_Wx);
    
    // precalaculations
    float r = sqrt(p_Wx*p_Wx + p_Wy*p_Wy);
    float s = p_Wz - DH_d1;
    
    // Check if position is within reach (workspace limits)
    float reach_distance = sqrt(r*r + s*s);
    float max_reach = DH_a2 + DH_a3;
    float min_reach = abs(DH_a2 - DH_a3);
    
    if (debug_enabled) {
        Serial.print(F("DBG: Wrist pos ("));
        Serial.print(p_Wx); Serial.print(',');
        Serial.print(p_Wy); Serial.print(',');
        Serial.print(p_Wz); Serial.println(')');
        Serial.print(F("DBG: r="));
        Serial.print(r); Serial.print(F(", s="));
        Serial.print(s); Serial.print(F(", reach_dist="));
        Serial.println(reach_distance);
        Serial.print(F("DBG: min_reach="));
        Serial.print(min_reach); Serial.print(F(", max_reach="));
        Serial.println(max_reach);
    }
    
    if (reach_distance > max_reach) {
        debug("IK error: position too far");
        return false;
    }
    
    if (reach_distance < min_reach) {
        debug("IK error: position too close");
        return false;
    }
    
    // Check for division by zero
    if (abs(2*DH_a2*DH_a3) < 0.001) {
        debug("IK error: zero denominator");
        return false;
    }
    
    // Calculate t3 with domain checking for acos
    float cos_t3_arg = (r*r + s*s - DH_a2*DH_a2 - DH_a3*DH_a3)/(2*DH_a2*DH_a3);
    if (cos_t3_arg > 1.0 || cos_t3_arg < -1.0) {
        debug("IK error: invalid acos argument");
        return false;
    }
    float t3 = acos(cos_t3_arg);

    // Check for division by zero for t2 calculation
    float denominator_t2 = r*r + s*s;
    if (denominator_t2 < 0.001) {
        debug("IK error: singularity at origin");
        return false;
    }
    
    // Calculate t2 with domain checking for asin
    float sin_t2_arg = ((DH_a2 + DH_a3*cos(t3))*s - DH_a3*sin(t3)*r) / denominator_t2;
    if (sin_t2_arg > 1.0 || sin_t2_arg < -1.0) {
        debug("IK error: invalid asin argument");
        return false;
    }
    float t2 = asin(sin_t2_arg);

    float t4 = atan2(-cos(t1)*sin(t2+t3)*T[0][2] - sin(t1)*sin(t2+t3)*T[1][2] + cos(t2+t3)*T[2][2], cos(t1)*cos(t2+t3)*T[0][2] + sin(t1)*cos(t2+t3)*T[1][2] + sin(t2+t3)*T[2][2]);

    // Check for valid t5 calculation (avoid sqrt of negative number)
    float sin_t1_T02_minus_cos_t1_T12 = sin(t1)*T[0][2] - cos(t1)*T[1][2];
    float sqrt_arg = 1 - sin_t1_T02_minus_cos_t1_T12 * sin_t1_T02_minus_cos_t1_T12;
    if (sqrt_arg < 0) {
        debug("IK error: invalid sqrt argument for t5");
        return false;
    }
    float t5 = atan2(sqrt(sqrt_arg), sin_t1_T02_minus_cos_t1_T12);

    // Fix typo: cost -> cos
    float t6 = atan2(sin(t1)*T[0][1] - cos(t1)*T[1][1], -sin(t1)*T[0][0] + cos(t1)*T[1][0]);

    result_angles[0] = SERVO_NEUTRALS[0] + t1 * RAD_TO_DEG;
    result_angles[1] = SERVO_NEUTRALS[1] + t2 * RAD_TO_DEG;
    result_angles[2] = SERVO_NEUTRALS[2] + t3 * RAD_TO_DEG;
    result_angles[3] = SERVO_NEUTRALS[3] + t4 * RAD_TO_DEG;
    result_angles[4] = SERVO_NEUTRALS[4] + t5 * RAD_TO_DEG;
    result_angles[5] = SERVO_NEUTRALS[5] + t6 * RAD_TO_DEG;

    // Debug: show calculated joint angles
    if (debug_enabled) {
        Serial.print(F("DBG: Joint angles: ["));
        for (int i = 0; i < 6; i++) {
            Serial.print(result_angles[i]);
            if (i < 5) Serial.print(F(", "));
        }
        Serial.println(F("]"));
    }
    
    // Check if resulting angles are within servo limits
    for (int i = 0; i < 6; i++) {
        if (result_angles[i] < SERVO_MIN_ANGLES[i] || result_angles[i] > SERVO_MAX_ANGLES[i]) {
            if (debug_enabled) {
                Serial.print(F("DBG: J"));
                Serial.print(i+1);
                Serial.print(F(" out of range: "));
                Serial.print(result_angles[i]);
                Serial.print(F("° (limit: "));
                Serial.print(SERVO_MIN_ANGLES[i]);
                Serial.print(F(" to "));
                Serial.print(SERVO_MAX_ANGLES[i]);
                Serial.println(F("°)"));
            }
            debug("IK error: joint limit exceeded");
            return false;
        }
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
            
            servos[i].write(logicalToServo(angles[i]));
        }
    }
    
    if (all_reached) {
        debug("Motion complete");
        is_moving = false;
    } else if (show_debug) {
        Serial.print(F("DBG: Moving J["));
        for (int i = 0; i < 7; i++) {
            Serial.print(angles[i]);
            if (i < 6) Serial.print(',');
        }
        Serial.println(']');
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
    if (limits == nullptr) return;
    
    // Calculate workspace limits for 6-DOF anthropomorphic arm
    // limits array: [x_min, x_max, y_min, y_max, z_min, z_max]
    
    // Maximum reach in XY plane (when arm is fully extended horizontally)
    float max_horizontal_reach = DH_a2 + DH_a3 + DH_d6;
    
    // Minimum reach in XY plane (when arm is folded inward)
    float min_horizontal_reach = abs(DH_a2 - DH_a3) - DH_d6;
    if (min_horizontal_reach < 0) min_horizontal_reach = 0;
    
    // X limits (symmetric around base due to 360° rotation of joint 1)
    limits[0] = -max_horizontal_reach;  // x_min
    limits[1] = max_horizontal_reach;   // x_max
    
    // Y limits (symmetric around base due to 360° rotation of joint 1)
    limits[2] = -max_horizontal_reach;  // y_min
    limits[3] = max_horizontal_reach;   // y_max
    
    // Z limits are more complex due to joint 2 and 3 constraints
    // Maximum height: when arm points straight up
    float max_height = DH_d1 + DH_a2 + DH_a3 + DH_d6;
    
    // Minimum height: when arm points down (limited by joint ranges)
    // Assuming joints can reach -90° to +90°, worst case is arm pointing down
    float min_height = DH_d1 - (DH_a2 + DH_a3 + DH_d6);
    
    // However, we need to account for actual servo limits
    // For safety, use more conservative estimates based on typical reachability
    float conservative_min_height = DH_d1 - (DH_a2 + DH_a3) * 0.8;
    float conservative_max_height = DH_d1 + (DH_a2 + DH_a3) * 0.9 + DH_d6;
    
    limits[4] = max(min_height, conservative_min_height);  // z_min
    limits[5] = min(max_height, conservative_max_height);  // z_max
    
    // Ensure minimum values don't exceed maximum values
    if (limits[4] > limits[5]) {
        float temp = limits[4];
        limits[4] = limits[5];
        limits[5] = temp;
    }
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
        Serial.println(F("OK Commands: HELP, STATE, MOVEJ, MOVE, JOG, GRIP, HOME, STOP, DEBUG"));
        Serial.println(F("Examples:"));
        Serial.println(F("  MOVEJ J2 45     // Move joint 2 to 45°"));
        Serial.println(F("  MOVE X 150 Y 0 Z 100 ROLL 0 PITCH 0 YAW 0"));
        Serial.println(F("  JOG J1 15       // Jog joint 1 by 15°"));
        Serial.println(F("  GRIP 30         // Set gripper to 30°"));
        Serial.println(F("Joint range: -90° to +90° (0° = neutral)"));
        
    } else if (cmd == "STATE") {
        printState();
        
    } else if (cmd == "HOME") {
        debug("Homing all joints");
        for (int i = 0; i < 7; i++) {
            setTargetJoint(i, SERVO_NEUTRALS[i]);
        }
        startSmoothMove();
        Serial.println(F("OK Moving home"));
        
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
        float x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;  // Default orientation
        
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
            else if (token == "ROLL") roll = getFloat(params);
            else if (token == "PITCH") pitch = getFloat(params);
            else if (token == "YAW") yaw = getFloat(params);
        }
        
        debugPos(x, y, z);
        if (debug_enabled) {
            Serial.print(F("DBG: Target orientation ("));
            Serial.print(roll); Serial.print(',');
            Serial.print(pitch); Serial.print(',');
            Serial.print(yaw); Serial.println(F(")"));
        }
        
        float result_angles[6];
        if (inverseKin6DOF(x, y, z, roll * DEG_TO_RAD, pitch * DEG_TO_RAD, yaw * DEG_TO_RAD, result_angles)) {
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
        servos[i].write(logicalToServo(SERVO_NEUTRALS[i]));
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