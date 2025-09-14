/*
 * ZRO_MEK 6-DOF Robotic Arm - Joint Control Only
 */

#include <Servo.h>
#include <math.h>

// ========== ROBOT CONFIGURATION ==========

// Servo configuration - 6 servos total (6 DOF)
constexpr uint8_t PINS[6] = { 3, 5, 6, 9, 10, 11 }; // Servo pins

// Servo direction mapping: 1 = normal, -1 = reversed
constexpr int8_t SERVO_DIRECTION[6] = {1, 1, -1, 1, -1, 1}; // Adjust per servo

// Kinematic joint limits in degrees
constexpr float JOINT_MIN_ANGLES[6] = {-135, -90, -90, -135, -90, -90};
constexpr float JOINT_MAX_ANGLES[6] = {135, 90, 90, 135, 90, 90};

// Motion control
constexpr float MOVE_SPEED = 100.0;   // degrees per second
constexpr float MAIN_LOOP_DELAY = 20; // 50Hz update rate for smooth motion

// ========== GLOBAL VARIABLES ==========
Servo servos[6];
float joint_angles[6];        // Current joint angles in kinematic degrees (0° = neutral)
float target_joint_angles[6]; // Target joint angles in kinematic degrees
String cmd = "";              // Command buffer
bool is_moving = false;       // Movement state
bool debug_enabled = false;   // Debug logging flag

// ========== UTILITY FUNCTIONS ==========
void debug(const char *msg)
{
    if (debug_enabled)
    {
        Serial.print(F("DBG: "));
        Serial.println(msg);
    }
}


// Convert kinematic angle to PWM angle for servo
// Maps each joint's kinematic range to servo write() range [0°, 180°]
// write(0) = minimum position, write(90) = center, write(180) = maximum position
// Returns -1 if angle is out of bounds
inline float kinematicToPWM(int joint, float kinematic_angle)
{
    // Apply direction mapping (reverse if needed)
    float mapped_angle = kinematic_angle * SERVO_DIRECTION[joint];

    // Check bounds AFTER applying direction mapping
    float min_angle = JOINT_MIN_ANGLES[joint];
    float max_angle = JOINT_MAX_ANGLES[joint];

    if (mapped_angle < min_angle || mapped_angle > max_angle)
    {
        return -1; // Invalid angle - out of bounds
    }

    // Map from [min_angle, max_angle] to [0, 180]
    float range = max_angle - min_angle;
    return (mapped_angle - min_angle) * (180.0 / range);
}

void setJoint(int j, float kinematic_angle)
{
    if (j >= 0 && j < 6)
    {
        float pwm_angle = kinematicToPWM(j, kinematic_angle);
        if (pwm_angle < 0)
        {
            // Angle is out of bounds - reject the operation
            if (debug_enabled)
            {
                Serial.print(F("DBG: Joint J"));
                Serial.print(j + 1);
                Serial.print(F(" angle "));
                Serial.print(kinematic_angle);
                Serial.print(F(" out of bounds ["));
                Serial.print(JOINT_MIN_ANGLES[j]);
                Serial.print(F(" to "));
                Serial.print(JOINT_MAX_ANGLES[j]);
                Serial.println(F("]"));
            }
            return; // Don't move the servo
        }

        joint_angles[j] = kinematic_angle;
        servos[j].write((int)pwm_angle);
    }
}

bool setTargetJoint(int j, float kinematic_angle)
{
    if (j >= 0 && j < 6)
    {
        float pwm_angle = kinematicToPWM(j, kinematic_angle);
        if (pwm_angle < 0)
        {
            // Angle is out of bounds - reject the target
            if (debug_enabled)
            {
                Serial.print(F("DBG: Target J"));
                Serial.print(j + 1);
                Serial.print(F(" angle "));
                Serial.print(kinematic_angle);
                Serial.print(F(" out of bounds ["));
                Serial.print(JOINT_MIN_ANGLES[j]);
                Serial.print(F(" to "));
                Serial.print(JOINT_MAX_ANGLES[j]);
                Serial.println(F("]"));
            }
            return false; // Target rejected
        }

        target_joint_angles[j] = kinematic_angle;
        return true; // Target accepted
    }
    return false;
}

void startSmoothMove()
{
    debug("Motion start");
    is_moving = true;
}

void updateSmoothMotion()
{
    if (!is_moving)
        return;

    bool all_reached = true;
    static unsigned long last_debug = 0;
    bool show_debug = debug_enabled && (millis() - last_debug > 500);

    for (int i = 0; i < 6; i++)
    {
        float diff = target_joint_angles[i] - joint_angles[i];

        if (fabs(diff) > 0.1)
        {
            all_reached = false;

            float step = MOVE_SPEED * (MAIN_LOOP_DELAY / 1000.0);
            if (fabs(diff) < step)
            {
                joint_angles[i] = target_joint_angles[i];
            }
            else
            {
                joint_angles[i] += (diff > 0) ? step : -step;
            }

            float pwm_angle = kinematicToPWM(i, joint_angles[i]);
            servos[i].write((int)pwm_angle);
        }
    }

    if (all_reached)
    {
        debug("Motion complete");
        is_moving = false;
    }
    else if (show_debug)
    {
        Serial.print(F("DBG: Moving J["));
        for (int i = 0; i < 6; i++)
        {
            Serial.print(joint_angles[i]);
            if (i < 5)
                Serial.print(',');
        }
        Serial.println(']');
        last_debug = millis();
    }
}

float getFloat(String &str)
{
    int idx = str.indexOf(' ');
    if (idx > 0)
    {
        String val = str.substring(0, idx);
        str = str.substring(idx + 1);
        return val.toFloat();
    }
    float val = str.toFloat();
    str = "";
    return val;
}

// ========== COMMAND PROCESSING ==========
void printState()
{
    Serial.println(F("OK {"));
    Serial.print(F("  \"joints\": ["));
    for (int i = 0; i < 6; i++)
    {
        Serial.print(joint_angles[i]);
        if (i < 5)
            Serial.print(F(", "));
    }
    Serial.println(F("]"));
    Serial.println(F("}"));
}

void processCmd()
{
    cmd.trim();
    cmd.toUpperCase();

    if (is_moving && cmd != "HELP" && cmd != "STATE" && cmd != "STOP" && !cmd.startsWith("DEBUG"))
    {
        debug("Command blocked - moving");
        Serial.println(F("ERR BUSY - Wait for current movement to finish"));
        return;
    }

    if (cmd == "HELP")
    {
        Serial.println(F("OK Commands: HELP, STATE, MOVEJ, JOG, HOME, STOP, DEBUG"));
        Serial.println(F("Examples:"));
        Serial.println(F("  MOVEJ J2 45     // Move joint 2 to 45°"));
        Serial.println(F("  JOG J1 15       // Jog joint 1 by 15°"));
        Serial.println(F("Joint range: varies by servo (0° = neutral pose)"));
    }
    else if (cmd == "STATE")
    {
        printState();
    }
    else if (cmd == "HOME")
    {
        debug("Homing all joints");
        bool all_valid = true;
        for (int i = 0; i < 6; i++)
        {
            if (!setTargetJoint(i, 0.0)) // Kinematic neutral = 0°
            {
                all_valid = false;
                Serial.print(F("ERR J"));
                Serial.print(i + 1);
                Serial.println(F(" cannot reach home position"));
            }
        }
        if (all_valid)
        {
            startSmoothMove();
            Serial.println(F("OK Moving home"));
        }
        else
        {
            Serial.println(F("ERR Home position invalid for some joints"));
        }
    }
    else if (cmd.startsWith("MOVEJ "))
    {
        String params = cmd.substring(6);
        int spaceIdx = params.indexOf(' ');
        if (spaceIdx == -1)
        {
            Serial.println(F("ERR Missing parameters"));
        }
        else
        {
            String joint = params.substring(0, spaceIdx);
            params = params.substring(spaceIdx + 1);

            if (joint.startsWith("J"))
            {
                int j = joint.substring(1).toInt() - 1;
                if (j >= 0 && j < 6)
                {
                    float kinematic_angle = getFloat(params);
                    if (debug_enabled)
                    {
                        Serial.print(F("DBG: MovJ target J"));
                        Serial.println(j + 1);
                    }

                    if (setTargetJoint(j, kinematic_angle))
                    {
                        startSmoothMove();
                        Serial.print(F("OK Moving J"));
                        Serial.print(j + 1);
                        Serial.print(F(" to "));
                        Serial.println(kinematic_angle);
                    }
                    else
                    {
                        Serial.print(F("ERR J"));
                        Serial.print(j + 1);
                        Serial.print(F(" angle "));
                        Serial.print(kinematic_angle);
                        Serial.println(F(" out of range"));
                    }
                }
                else
                {
                    Serial.println(F("ERR Invalid joint"));
                }
            }
            else
            {
                Serial.println(F("ERR Invalid joint format"));
            }
        }
    }
    else if (cmd.startsWith("JOG "))
    {
        String params = cmd.substring(4);
        int spaceIdx = params.indexOf(' ');
        if (spaceIdx == -1)
        {
            Serial.println(F("ERR Missing parameters"));
        }
        else
        {
            String joint = params.substring(0, spaceIdx);
            params = params.substring(spaceIdx + 1);

            if (joint.startsWith("J"))
            {
                int j = joint.substring(1).toInt() - 1;
                if (j >= 0 && j < 6)
                {
                    float jog_amount = getFloat(params);
                    float current_angle = joint_angles[j];
                    float requested_angle = current_angle + jog_amount;

                    if (debug_enabled)
                    {
                        Serial.print(F("DBG: Jog J"));
                        Serial.print(j + 1);
                        Serial.print(F(" by "));
                        Serial.println(jog_amount);
                    }

                    if (setTargetJoint(j, requested_angle))
                    {
                        startSmoothMove();
                        Serial.print(F("OK Jogging J"));
                        Serial.print(j + 1);
                        Serial.print(F(" to "));
                        Serial.println(requested_angle);
                    }
                    else
                    {
                        Serial.print(F("ERR J"));
                        Serial.print(j + 1);
                        Serial.print(F(" target "));
                        Serial.print(requested_angle);
                        Serial.println(F(" out of range"));
                    }
                }
                else
                {
                    Serial.println(F("ERR Invalid joint"));
                }
            }
            else
            {
                Serial.println(F("ERR Invalid joint format"));
            }
        }
    }
    else if (cmd == "STOP")
    {
        debug("Emergency stop");
        is_moving = false;
        Serial.println(F("OK Movement stopped"));
    }
    else if (cmd == "DEBUG ON")
    {
        debug_enabled = true;
        Serial.println(F("OK Debug enabled"));
    }
    else if (cmd == "DEBUG OFF")
    {
        debug_enabled = false;
        Serial.println(F("OK Debug disabled"));
    }
    else if (cmd == "DEBUG")
    {
        Serial.println(debug_enabled ? F("OK Debug ON") : F("OK Debug OFF"));
    }
    else
    {
        Serial.println(F("ERR Unknown command"));
    }
}

// ========== ARDUINO SETUP/LOOP ==========
void setup()
{
    Serial.begin(115200);
    Serial.println(F("ZRO_MEK 6-DOF Robotic Arm - Ready"));

    for (int i = 0; i < 6; i++)
    {
        servos[i].attach(PINS[i]);
        joint_angles[i] = 0.0;
        target_joint_angles[i] = 0.0;
        float pwm_angle = kinematicToPWM(i, 0.0);
        servos[i].write((int)pwm_angle);
        delay(100);
    }

    Serial.println(F("Type HELP for commands"));
}

void loop()
{
    // Handle serial commands
    while (Serial.available())
    {
        char c = Serial.read();
        if (c == '\n' || c == '\r')
        {
            if (cmd.length() > 0)
            {
                processCmd();
                cmd = "";
            }
        }
        else if (c >= 32)
        {
            cmd += c;
        }
    }

    // Update smooth motion
    updateSmoothMotion();

    delay(MAIN_LOOP_DELAY);
}