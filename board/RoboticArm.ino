/*
 * ZRO_MEK 6-DOF Anthropomorphic Arm with Spherical Wrist + Gripper
 * Based on classical Denavit-Hartenberg convention
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

// ========== CONFIGURABLE DH KINEMATICS ==========
// Forward kinematics for Standard Anthropomorphic Arm with Spherical Wrist
void forwardKin6DOF(float* pos, float* rot) {
// TODO
}


bool inverseKin6DOF(float x, float y, float z, float roll, float pitch, float yaw, float* result_angles) {

    float T[4][4];
    makeTransformFromRPY(x, y,z, roll, pitch, yaw, T);

    float p_Wx = x - DH_d6 * T[0][2];
    float p_Wy = y - DH_d6 * T[1][2];
    float p_Wz = z - DH_d6 * T[2][2];
    
    float t1 = atan2(p_Wy, p_Wx);
    
    // precalaculations
    float r = sqrt(p_Wx*p_Wx + p_Wy*p_Wy);
    float s = p_Wz - DH_d1;

    float t3 = acos((r*r + s*s - DH_a2*DH_a2 - DH_a3*DH_a3)/(2*DH_a2*DH_a3));

    float t2 = asin(((DH_a2 + DH_a3*cos(t3))*s - DH_a3*sin(t3)*r) / (r*r + s*s));

    float t4 = atan2(-cos(t1)*sin(t2+t3)*T[0][2] - sin(t1)*sin(t2+t3)*T[1][2] + cos(t2+t3)*T[2][2], cos(t1)*cos(t2+t3)*T[0][2] + sin(t1)*cos(t2+t3)*T[1][2] + sin(t2+t3)*T[2][2]);

    float t5 = atan2(sqrt(1 - (sin(t1)*T[0][2] - cos(t1)*T[1][2])*(sin(t1)*T[0][2] - cos(t1)*T[1][2])), sin(t1)*T[0][2] - cos(t1)*T[1][2]);

    float t6 = atan2(sin(t1)*T[0][1] - cos(t1)*T[1][1], -sin(t1)*T[0][1] + cos(t1)*T[1][1]);


    result_angles[0] = SERVO_NEUTRALS[0] + t1 * RAD_TO_DEG;
    result_angles[1] = SERVO_NEUTRALS[1] + t2 * RAD_TO_DEG;
    result_angles[2] = SERVO_NEUTRALS[2] + t3 * RAD_TO_DEG;
    result_angles[3] = SERVO_NEUTRALS[3] + t4 * RAD_TO_DEG;
    result_angles[4] = SERVO_NEUTRALS[4] + t5 * RAD_TO_DEG;
    result_angles[5] = SERVO_NEUTRALS[5] + t6 * RAD_TO_DEG;
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
 // TODO
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
        Serial.println(F("  MOVE X 150 Y 0 Z 100 AX 1 AY 0 AZ 0"));
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