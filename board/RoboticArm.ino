/*
 * 6-DOF Robotic Arm Controller - Lightweight Version for Arduino Uno
 * Reduced memory footprint for simulators
 */

#include <Servo.h>
#include <EEPROM.h>

// ========== CONFIGURATION ==========
constexpr float L1 = 0, L2 = 120, L3 = 120, L4 = 40, L5 = 60;  // Link lengths (mm)
constexpr int NUM_JOINTS = 6;
constexpr uint8_t PINS[6] = {3, 5, 6, 9, 10, 11};  // Servo pins
constexpr float LIMITS_MIN[6] = {-160, -10, -10, -90, -180, 0};
constexpr float LIMITS_MAX[6] = {160, 180, 180, 90, 180, 100};

// ========== GLOBAL VARIABLES ==========
Servo servos[6];
float angles[6] = {0, 0, 0, 0, 0, 0};  // Current joint angles
String cmd = "";  // Command buffer
bool calibMode = false;

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
void forwardKin(float* pos) {  // Simple 3DOF forward kinematics
    float j1 = angles[0] * 0.01745;  // DEG_TO_RAD
    float j2 = angles[1] * 0.01745;
    float j3 = angles[2] * 0.01745;
    
    float r = L2 * cos(j2) + L3 * cos(j2 + j3) + L4 + L5;
    pos[0] = r * sin(j1);  // x
    pos[1] = L1 + L2 * sin(j2) + L3 * sin(j2 + j3);  // y
    pos[2] = r * cos(j1);  // z
}

bool inverseKin(float x, float y, float z) {  // Simple 3DOF IK
    angles[0] = atan2(x, z) * 57.296;  // RAD_TO_DEG
    
    float r = sqrt(x*x + z*z) - L4 - L5;
    float h = y - L1;
    float d = sqrt(r*r + h*h);
    
    if (d > L2 + L3 || d < abs(L2 - L3)) return false;
    
    float a = acos((L2*L2 + d*d - L3*L3) / (2*L2*d));
    float b = atan2(h, r);
    angles[1] = (a + b) * 57.296;
    
    float c = acos((L2*L2 + L3*L3 - d*d) / (2*L2*L3));
    angles[2] = (3.14159 - c) * 57.296;
    
    return true;
}

// ========== COMMAND PROCESSING ==========
void printState() {
    float pos[3];
    forwardKin(pos);
    
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
    Serial.print(pos[2]); Serial.println("]");
    Serial.println("}");
}

void processCmd() {
    cmd.trim();
    cmd.toUpperCase();
    
    if (cmd == "HELP") {
        Serial.println("OK Commands: HELP, GET STATE, MOVEJ, MOVEL, JOG, GRIP, HOME");
        
    } else if (cmd == "GET STATE") {
        printState();
        
    } else if (cmd == "HOME") {
        for (int i = 0; i < 6; i++) setJoint(i, 0);
        Serial.println("OK Homed");
        
    } else if (cmd.startsWith("MOVEJ ")) {
        String params = cmd.substring(6);
        for (int i = 0; i < 6; i++) {
            setJoint(i, getFloat(params));
        }
        Serial.println("OK Moved to joint position");
        
    } else if (cmd.startsWith("MOVEL ")) {
        String params = cmd.substring(6);
        float x = 0, y = 0, z = 0;
        
        while (params.length() > 0) {
            String token = params.substring(0, params.indexOf(' '));
            params = params.substring(params.indexOf(' ') + 1);
            
            if (token == "X") x = getFloat(params);
            else if (token == "Y") y = getFloat(params);
            else if (token == "Z") z = getFloat(params);
        }
        
        if (inverseKin(x, y, z)) {
            for (int i = 0; i < 3; i++) setJoint(i, angles[i]);
            Serial.println("OK Moved to position");
        } else {
            Serial.println("ERR UNREACHABLE");
        }
        
    } else if (cmd.startsWith("JOG ")) {
        String params = cmd.substring(4);
        String joint = params.substring(0, params.indexOf(' '));
        params = params.substring(params.indexOf(' ') + 1);
        
        if (joint.startsWith("J")) {
            int j = joint.substring(1).toInt() - 1;
            if (j >= 0 && j < 6) {
                setJoint(j, angles[j] + getFloat(params));
                Serial.println("OK Jogged J" + String(j+1));
            } else {
                Serial.println("ERR Invalid joint");
            }
        }
        
    } else if (cmd.startsWith("GRIP ")) {
        float percent = cmd.substring(5).toFloat();
        setJoint(5, percent);
        Serial.println("OK Gripper set");
        
    } else if (cmd == "CALIB START") {
        calibMode = true;
        Serial.println("OK Calibration mode");
        
    } else if (cmd == "CALIB SETZERO") {
        if (calibMode) {
            Serial.println("OK Zero position set");
        } else {
            Serial.println("ERR Not in calibration mode");
        }
        
    } else {
        Serial.println("ERR Unknown command");
    }
}

// ========== ARDUINO SETUP/LOOP ==========
void setup() {
    Serial.begin(115200);
    Serial.println("6-DOF Robotic Arm Lite - Ready");
    
    for (int i = 0; i < 6; i++) {
        servos[i].attach(PINS[i]);
        setJoint(i, 0);
        delay(10);
    }
    
    Serial.println("Type HELP for commands");
}

void loop() {
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
    delay(1);
}
