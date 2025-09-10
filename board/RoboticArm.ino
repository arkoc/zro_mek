/*
 * ZRO_MEK 6-DOF Anthropomorphic Arm with Spherical Wrist
 */

#include <Servo.h>
#include <math.h>

// ========== ROBOT CONFIGURATION ==========
// DH Parameters (modify these according to your robot)
const float a2 = 90;    // Upper arm length
const float a3 = 200;    // Forearm length
const float d1 = 130;    // Base height
const float d6 = 120;   // End effector length

// Servo configuration - 6 servos total (6 DOF)
constexpr uint8_t PINS[6] = {3, 5, 6, 9, 10, 11};  // Servo pins

// Kinematic joint limits in degrees
constexpr float JOINT_MIN_ANGLES[6] = {-135, -90, -90, -135, -90, -135};  // 270° servo: ±135°, 180° servo: ±90°
constexpr float JOINT_MAX_ANGLES[6] = {135, 90, 90, 135, 90, 135};

// Motion control
constexpr float MOVE_SPEED = 100.0;  // degrees per second  
constexpr float MAIN_LOOP_DELAY = 20; // 50Hz update rate for smooth motion

// ========== GLOBAL VARIABLES ==========
Servo servos[6];
float joint_angles[6];         // Current joint angles in kinematic degrees (0° = neutral)
float target_joint_angles[6];  // Target joint angles in kinematic degrees
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
        Serial.print(x); Serial.print(',');
        Serial.print(y); Serial.print(',');
        Serial.print(z); Serial.println(')');
    }
}

// Convert kinematic angle to PWM angle for servo
inline float kinematicToPWM(int joint, float kinematic_angle) {
    return kinematic_angle + JOINT_MAX_ANGLES[joint];
}

void setJoint(int j, float kinematic_angle) {
    if (j >= 0 && j < 6) {
        float orig_angle = kinematic_angle;
        kinematic_angle = constrain(kinematic_angle, JOINT_MIN_ANGLES[j], JOINT_MAX_ANGLES[j]);
        if (orig_angle != kinematic_angle && debug_enabled) {
            Serial.print(F("DBG: Joint limit J"));
            Serial.println(j+1);
        }
        joint_angles[j] = kinematic_angle;
        float pwm_angle = kinematicToPWM(j, kinematic_angle);
        servos[j].write((int)pwm_angle);
    }
}

void setTargetJoint(int j, float kinematic_angle) {
    if (j >= 0 && j < 6) {
        target_joint_angles[j] = constrain(kinematic_angle, JOINT_MIN_ANGLES[j], JOINT_MAX_ANGLES[j]);
    }
}

void startSmoothMove() {
    debug("Motion start");
    is_moving = true;
}

void getTransformation(float T[4][4], float x, float y, float z, float roll, float pitch, float yaw) {
  float cy = cos(roll);
  float sy = sin(roll);
  float cb = cos(pitch);
  float sb = sin(pitch);
  float ca = cos(yaw);
  float sa = sin(yaw);
  
  // Row 0
  T[0][0] = cb * cy;
  T[0][1] = sa * sb * cy - ca * sy;
  T[0][2] = ca * sb * cy + sa * sy;
  T[0][3] = x;
  
  // Row 1
  T[1][0] = cb * sy;
  T[1][1] = sa * sb * sy + ca * cy;
  T[1][2] = ca * sb * sy - sa * cy;
  T[1][3] = y;
  
  // Row 2
  T[2][0] = -sb;
  T[2][1] = sa * cb;
  T[2][2] = ca * cb;
  T[2][3] = z;
  
  // Row 3
  T[3][0] = 0;
  T[3][1] = 0;
  T[3][2] = 0;
  T[3][3] = 1;
}


void forward_kinematics(float joint_angles[6], float* x, float* y, float* z, float* roll, float* pitch, float* yaw) {
    float t1 = (joint_angles[0]) * DEG_TO_RAD;
    float t2 = (joint_angles[1]) * DEG_TO_RAD;
    float t3 = (joint_angles[2]) * DEG_TO_RAD;
    float t4 = (joint_angles[3]) * DEG_TO_RAD;
    float t5 = (joint_angles[4]) * DEG_TO_RAD;
    float t6 = (joint_angles[5]) * DEG_TO_RAD;
    
    // Precompute trigonometric values
    float c1 = cos(t1), s1 = sin(t1);
    float c2 = cos(t2), s2 = sin(t2);
    float c3 = cos(t3), s3 = sin(t3);
    float c4 = cos(t4), s4 = sin(t4);
    float c5 = cos(t5), s5 = sin(t5);
    float c6 = cos(t6), s6 = sin(t6);
    
    float c23 = cos(t2 + t3), s23 = sin(t2 + t3);
    
    // Build transformation matrix T0_6 = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6
    float T[4][4];
    
    // Rotation matrix components
    T[0][0] = c1*(c23*(c4*c5*c6 - s4*s6) - s23*s5*c6) - s1*(s4*c5*c6 + c4*s6);
    T[0][1] = -c1*(c23*(c4*c5*s6 + s4*c6) - s23*s5*s6) + s1*(s4*c5*s6 - c4*c6);
    T[0][2] = c1*(c23*c4*s5 + s23*c5) - s1*s4*s5;
    
    T[1][0] = s1*(c23*(c4*c5*c6 - s4*s6) - s23*s5*c6) + c1*(s4*c5*c6 + c4*s6);
    T[1][1] = -s1*(c23*(c4*c5*s6 + s4*c6) - s23*s5*s6) - c1*(s4*c5*s6 - c4*c6);
    T[1][2] = s1*(c23*c4*s5 + s23*c5) + c1*s4*s5;
    
    T[2][0] = -s23*(c4*c5*c6 - s4*s6) - c23*s5*c6;
    T[2][1] = s23*(c4*c5*s6 + s4*c6) + c23*s5*s6;
    T[2][2] = -s23*c4*s5 + c23*c5;
    
    // Position vector (translation)
    T[0][3] = c1*(a2*c2 + a3*c23) - d6*T[0][2];
    T[1][3] = s1*(a2*c2 + a3*c23) - d6*T[1][2];
    T[2][3] = d1 + a2*s2 + a3*s23 - d6*T[2][2];
    
    // Homogeneous coordinates
    T[3][0] = T[3][1] = T[3][2] = 0;
    T[3][3] = 1;
    
    // Extract position
    *x = T[0][3];
    *y = T[1][3];
    *z = T[2][3];
    
    // Extract orientation (ZYX Euler angles)
    // Roll (rotation about X-axis)
    *roll = atan2(T[2][1], T[2][2]);
    
    // Pitch (rotation about Y-axis)
    *pitch = atan2(-T[2][0], sqrt(T[2][1]*T[2][1] + T[2][2]*T[2][2]));
    
    // Yaw (rotation about Z-axis)
    *yaw = atan2(T[1][0], T[0][0]);
    
    // Convert angles back to degrees
    *roll *= RAD_TO_DEG;
    *pitch *= RAD_TO_DEG;
    *yaw *= RAD_TO_DEG;
}

bool inverse_kinematics(float x, float y, float z, float roll, float pitch, float yaw, float* result_angles) {

    float solutions[8][6];

    // Get all 8 solutions
    inverse_kinematics_all_solutions(x, y, z, roll, pitch, yaw, solutions);
    
    // Check each solution against servo limits
    for (int i = 0; i < 8; i++) {
        bool valid = true;
        
        // Check if all angles are within joint limits
        for (int j = 0; j < 6; j++) {
            if (solutions[i][j] < JOINT_MIN_ANGLES[j] || solutions[i][j] > JOINT_MAX_ANGLES[j]) {
                valid = false;
                break;
            }
        }
        
        // If this solution is valid, copy it to result and return true
        if (valid) {
            for (int j = 0; j < 6; j++) {
                result_angles[j] = solutions[i][j];
            }
            return true;
        }
    }
    
    // No valid solution found within servo limits
    return false;
}

void inverse_kinematics_all_solutions(float x, float y, float z, float roll, float pitch, float yaw, float solutions[8][6]) {
    float T[4][4];
    getTransformation(T, x, y, z, roll * DEG_TO_RAD, pitch * DEG_TO_RAD, yaw * DEG_TO_RAD);
    
    // Calculate wrist center position
    float p_Wx = x - d6 * T[0][2];
    float p_Wy = y - d6 * T[1][2];
    float p_Wz = z - d6 * T[2][2];
    
    // Wrist center calculations
    float r = sqrt(p_Wx*p_Wx + p_Wy*p_Wy);
    float s = p_Wz - d1;
    
    int solution_idx = 0;
    
    // Two solutions for theta1 (±180°)
    for (int i1 = 0; i1 < 2; i1++) {
        float t1 = atan2(p_Wy, p_Wx);
        if (i1 == 1) {
            t1 += M_PI;  // Add 180 degrees for second solution
        }
        
        // Two solutions for theta3 (elbow up/down)
        for (int i3 = 0; i3 < 2; i3++) {
            float cos_t3 = (r*r + s*s - a2*a2 - a3*a3)/(2*a2*a3);
            float t3 = acos(cos_t3);
            if (i3 == 1) {
                t3 = -t3;  // Negative angle for elbow down
            }
            
            float t2 = asin(((a2 + a3*cos(t3))*s - a3*sin(t3)*r) / (r*r + s*s));    

            // Two solutions for theta5 (wrist up/down)
            for (int i5 = 0; i5 < 2; i5++) {
                float cos_t5 = sin(t1)*T[0][2] - cos(t1)*T[1][2];
                float sin_t5 = sqrt(1 - cos_t5*cos_t5);
                if (i5 == 1) {
                    sin_t5 = -sin_t5;  // Negative for wrist down
                }
                
                float t4 = atan2(-cos(t1)*sin(t2+t3)*T[0][2] - sin(t1)*sin(t2+t3)*T[1][2] + cos(t2+t3)*T[2][2], 
                                 cos(t1)*cos(t2+t3)*T[0][2] + sin(t1)*cos(t2+t3)*T[1][2] + sin(t2+t3)*T[2][2]);
                
                float t5 = atan2(sin_t5, cos_t5);
                
                float t6 = atan2(sin(t1)*T[0][1] - cos(t1)*T[1][1], -sin(t1)*T[0][0] + cos(t1)*T[1][0]);
                
                // Store solution
                solutions[solution_idx][0] = t1 * RAD_TO_DEG;
                solutions[solution_idx][1] = t2 * RAD_TO_DEG;
                solutions[solution_idx][2] = t3 * RAD_TO_DEG;
                solutions[solution_idx][3] = t4 * RAD_TO_DEG;
                solutions[solution_idx][4] = t5 * RAD_TO_DEG;
                solutions[solution_idx][5] = t6 * RAD_TO_DEG;
                
                solution_idx++;
            }
        }
    }
}


bool checkJointLimits(float kinematic_angles[6]) {
  for (int i = 0; i < 6; i++) {
    if (kinematic_angles[i] < JOINT_MIN_ANGLES[i] || kinematic_angles[i] > JOINT_MAX_ANGLES[i]) {
      if (debug_enabled) {
        Serial.print(F("DBG: J"));
        Serial.print(i+1);
        Serial.print(F(" out of range: "));
        Serial.print(kinematic_angles[i]);
        Serial.print(F("° (limit: "));
        Serial.print(JOINT_MIN_ANGLES[i]);
        Serial.print(F(" to "));
        Serial.print(JOINT_MAX_ANGLES[i]);
        Serial.println(F("°)"));
      }
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
    
    for (int i = 0; i < 6; i++) {
        float diff = target_joint_angles[i] - joint_angles[i];
        
        if (fabs(diff) > 0.1) {
            all_reached = false;
            
            float step = MOVE_SPEED * (MAIN_LOOP_DELAY / 1000.0);
            if (fabs(diff) < step) {
                joint_angles[i] = target_joint_angles[i];
            } else {
                joint_angles[i] += (diff > 0) ? step : -step;
            }
            
            float pwm_angle = kinematicToPWM(i, joint_angles[i]);
            servos[i].write((int)pwm_angle);
        }
    }
    
    if (all_reached) {
        debug("Motion complete");
        is_moving = false;
    } else if (show_debug) {
        Serial.print(F("DBG: Moving J["));
        for (int i = 0; i < 6; i++) {
            Serial.print(joint_angles[i]);
            if (i < 5) Serial.print(',');
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

// ========== COMMAND PROCESSING ==========
void printState() {
    // Calculate forward kinematics
    float x, y, z, roll, pitch, yaw;
    forward_kinematics(joint_angles, &x, &y, &z, &roll, &pitch, &yaw);
    
    // Calculate workspace limits
    float max_reach = a2 + a3;
    
    Serial.println(F("OK {"));
    Serial.print(F("  \"joints\": ["));
    for (int i = 0; i < 6; i++) {
        Serial.print(joint_angles[i]);
        if (i < 5) Serial.print(F(", "));
    }
    Serial.println(F("],"));
    Serial.print(F("  \"pos\": ["));
    Serial.print(x); Serial.print(F(", "));
    Serial.print(y); Serial.print(F(", "));
    Serial.print(z); Serial.println(F("],"));
    Serial.print(F("  \"approach\": ["));
    Serial.print(roll); Serial.print(F(", "));
    Serial.print(pitch); Serial.print(F(", "));
    Serial.print(yaw); Serial.println(F("],"));
    Serial.print(F("  \"workspace\": {"));
    Serial.print(F("\"x\": [")); Serial.print(-max_reach); Serial.print(F(", ")); Serial.print(max_reach); Serial.print(F("], "));
    Serial.print(F("\"y\": [")); Serial.print(-max_reach); Serial.print(F(", ")); Serial.print(max_reach); Serial.print(F("], "));
    Serial.print(F("\"z\": [")); Serial.print(d1 - max_reach); Serial.print(F(", ")); Serial.print(d1 + max_reach); Serial.println(F("]}"));
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
        Serial.println(F("OK Commands: HELP, STATE, MOVEJ, MOVE, JOG, HOME, STOP, DEBUG"));
        Serial.println(F("Examples:"));
        Serial.println(F("  MOVEJ J2 45     // Move joint 2 to 45°"));
        Serial.println(F("  MOVE X 150 Y 0 Z 100 [ROLL 0] [PITCH 0] [YAW 0]"));
        Serial.println(F("  JOG J1 15       // Jog joint 1 by 15°"));
        Serial.println(F("Joint range: varies by servo (0° = neutral pose)"));
        
    } else if (cmd == "STATE") {
        printState();
        
    } else if (cmd == "HOME") {
        debug("Homing all joints");
        for (int i = 0; i < 6; i++) {
            setTargetJoint(i, 0.0);  // Kinematic neutral = 0°
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
                    float kinematic_angle = getFloat(params);
                    float limited_angle = constrain(kinematic_angle, JOINT_MIN_ANGLES[j], JOINT_MAX_ANGLES[j]);
                    if (debug_enabled) {
                        Serial.print(F("DBG: MovJ target J"));
                        Serial.println(j+1);
                    }
                    setTargetJoint(j, limited_angle);
                    startSmoothMove();
                    
                    Serial.print(F("OK "));
                    if (limited_angle != kinematic_angle) {
                        Serial.print(F("J"));
                        Serial.print(j+1);
                        Serial.print(F(" limit reached at "));
                        Serial.println(limited_angle);
                    } else {
                        Serial.print(F("Moving J"));
                        Serial.print(j+1);
                        Serial.print(F(" to "));
                        Serial.println(kinematic_angle);
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
        if (params.length() == 0) {
            Serial.println(F("ERR Missing parameters - Usage: MOVE X val Y val Z val [ROLL val] [PITCH val] [YAW val]"));
            return;
        }
        
        float x = NAN, y = NAN, z = NAN, roll = 0, pitch = 0, yaw = 0;
        bool x_set = false, y_set = false, z_set = false;
        
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
            
            if (token == "X") {
                if (params.length() == 0) {
                    Serial.println(F("ERR Missing value for X parameter"));
                    return;
                }
                x = getFloat(params);
                x_set = true;
            } else if (token == "Y") {
                if (params.length() == 0) {
                    Serial.println(F("ERR Missing value for Y parameter"));
                    return;
                }
                y = getFloat(params);
                y_set = true;
            } else if (token == "Z") {
                if (params.length() == 0) {
                    Serial.println(F("ERR Missing value for Z parameter"));
                    return;
                }
                z = getFloat(params);
                z_set = true;
            } else if (token == "ROLL") {
                if (params.length() == 0) {
                    Serial.println(F("ERR Missing value for ROLL parameter"));
                    return;
                }
                roll = getFloat(params);
            } else if (token == "PITCH") {
                if (params.length() == 0) {
                    Serial.println(F("ERR Missing value for PITCH parameter"));
                    return;
                }
                pitch = getFloat(params);
            } else if (token == "YAW") {
                if (params.length() == 0) {
                    Serial.println(F("ERR Missing value for YAW parameter"));
                    return;
                }
                yaw = getFloat(params);
            } else if (token.length() > 0) {
                Serial.print(F("ERR Invalid parameter: "));
                Serial.println(token);
                return;
            }
        }
        
        // Check parameter completeness
        if (!x_set || !y_set || !z_set) {
            Serial.println(F("ERR Missing required parameters - X, Y, and Z are required"));
            return;
        }
        
        // Validate parameter ranges
        float max_reach = a2 + a3;
        if (sqrt(x*x + y*y) > max_reach) {
            Serial.println(F("ERR Position out of workspace - XY distance too large"));
            return;
        }
        
        if (z < (d1 - max_reach) || z > (d1 + max_reach)) {
            Serial.println(F("ERR Position out of workspace - Z height invalid"));
            return;
        }
        
        if (roll < -180 || roll > 180 || pitch < -180 || pitch > 180 || yaw < -180 || yaw > 180) {
            Serial.println(F("ERR Orientation angles must be between -180 and 180 degrees"));
            return;
        }
        
        debugPos(x, y, z);
        if (debug_enabled) {
            Serial.print(F("DBG: Target orientation ("));
            Serial.print(roll); Serial.print(',');
            Serial.print(pitch); Serial.print(',');
            Serial.print(yaw); Serial.println(F(")"));
        }
        
        // Calculate inverse kinematics
        float result_angles[6];
        if (inverse_kinematics(x, y, z, roll, pitch, yaw, result_angles)) {
            debug("IK success");
            for (int i = 0; i < 6; i++) setTargetJoint(i, result_angles[i]);
            startSmoothMove();
            Serial.println(F("OK Moving to pose"));
        } else {
            Serial.println(F("ERR UNREACHABLE - No solution"));
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
                    float current_angle = joint_angles[j];
                    float requested_angle = current_angle + jog_amount;
                    float limited_angle = constrain(requested_angle, JOINT_MIN_ANGLES[j], JOINT_MAX_ANGLES[j]);
                    
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
    
    for (int i = 0; i < 6; i++) {
        servos[i].attach(PINS[i]);
        joint_angles[i] = 0.0;  // Kinematic neutral
        target_joint_angles[i] = 0.0;
        float pwm_angle = kinematicToPWM(i, 0.0);
        servos[i].write((int)pwm_angle);
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