#include <SimpleFOC.h>
#include <SPI.h>
#include "config.h"
#include "motor.h"
#include "network.h" 

// --- Motor Objects ---
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_EN);
MagneticSensorSPI sensor = MagneticSensorSPI(SENSOR_CS, 14, 0x3FFF);
PIDController haptic_pid = PIDController(0, 0, 0, 100000, POWER_SUPPLY_VOLTAGE);

// --- State Management Variables ---
KnobMode current_mode = MODE_LIGHT_BRIGHTNESS;
const char* mode_names[] = {"Light Brightness", "Media Volume", "Fan Speed", "Temperature", "Media Control", "Light Color"};
float mode_angles[NUM_MODES] = {0.0f};
bool is_transitioning = false;
float transition_target_angle = 0.0f;
float current_attractor_angle = 0.0f;

int8_t last_media_action = 0;
int current_color_index = 0;

// --- Helper Functions ---
float findDetent(float angle, float detent_width) {
    return round(angle / detent_width) * detent_width;
}

// --- Public Functions ---
void motor_init() {
    SPI.begin();
    sensor.init();
    motor.linkSensor(&sensor);
    motor.sensor_direction = Direction::CW; 
    
    driver.voltage_power_supply = POWER_SUPPLY_VOLTAGE;
    driver.init();
    motor.linkDriver(&driver);
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.controller = MotionControlType::torque;
    motor.init();
    motor.initFOC();
    motor_set_mode(MODE_LIGHT_BRIGHTNESS, true); 
    Serial.println("Motor Initialized.");
}

void motor_update() {
    motor.loopFOC();
    float voltage = 0;
    current_attractor_angle = motor.shaft_angle;

    if (is_transitioning) {
        voltage = haptic_pid(transition_target_angle - motor.shaft_angle);
        if (abs(transition_target_angle - motor.shaft_angle) < 0.025) {
            is_transitioning = false;
            motor_set_mode(current_mode, true);
        }
    } else {
        float angle_rad = motor.shaft_angle;
        float angle_deg = angle_rad * RAD_TO_DEG;

        switch (current_mode) {
            case MODE_LIGHT_BRIGHTNESS:
            case MODE_MEDIA_VOLUME: {
                if (angle_deg > ANALOG_MODE_MAX_ANGLE) {
                    current_attractor_angle = ANALOG_MODE_MAX_ANGLE * DEG_TO_RAD;
                    voltage = haptic_pid(current_attractor_angle - angle_rad);
                } else if (angle_deg < 0) {
                     current_attractor_angle = 0;
                     voltage = haptic_pid(current_attractor_angle - angle_rad);
                } else {
                    voltage = -HAPTIC_KD_DAMPING * motor.shaft_velocity;
                }
                break;
            }
            case MODE_FAN_SPEED: {
                current_attractor_angle = findDetent(angle_rad, FAN_DETENT_ANGLE * DEG_TO_RAD);
                voltage = haptic_pid(current_attractor_angle - angle_rad);
                break;
            }
            case MODE_TEMPERATURE_CONTROL: {
                 float min_rad = TEMP_MIN_ANGLE * DEG_TO_RAD;
                 float max_rad = TEMP_MAX_ANGLE * DEG_TO_RAD;

                 if (angle_rad > max_rad) {
                    current_attractor_angle = max_rad;
                    voltage = haptic_pid(current_attractor_angle - angle_rad);
                 } else if (angle_rad < min_rad) {
                    current_attractor_angle = min_rad;
                    voltage = haptic_pid(current_attractor_angle - angle_rad);
                 } else {
                    float relative_angle = angle_rad - min_rad;
                    float detent_width_rad = TEMP_DETENT_ANGLE * DEG_TO_RAD;
                    current_attractor_angle = findDetent(relative_angle, detent_width_rad) + min_rad;
                    voltage = haptic_pid(current_attractor_angle - angle_rad);
                 }
                 break;
            }
            case MODE_MEDIA_CONTROL: {
                float min_rad = TRACK_MODE_MIN_ANGLE * DEG_TO_RAD;
                float max_rad = TRACK_MODE_MAX_ANGLE * DEG_TO_RAD;
                
                if (angle_rad > max_rad) {
                    current_attractor_angle = max_rad;
                    voltage = haptic_pid(current_attractor_angle - angle_rad);
                    if (last_media_action != 1) {
                        network_publish_media_control("NEXT");
                        last_media_action = 1;
                    }
                } else if (angle_rad < min_rad) {
                    current_attractor_angle = min_rad;
                    voltage = haptic_pid(current_attractor_angle - angle_rad);
                    if (last_media_action != -1) {
                        network_publish_media_control("PREVIOUS");
                        last_media_action = -1;
                    }
                } else {
                    voltage = -HAPTIC_KD_DAMPING * motor.shaft_velocity;
                    last_media_action = 0;
                }
                break;
            }
            // --- FIX: Restored 360-degree spinner feel for Color Mode ---
            case MODE_LIGHT_COLOR: {
                // Wrap angle to be always positive for 360 degree control
                float wrapped_angle = fmod(angle_rad, 2 * PI);
                if (wrapped_angle < 0) {
                    wrapped_angle += 2 * PI;
                }
                current_attractor_angle = findDetent(wrapped_angle, COLOR_MODE_DETENT_ANGLE * DEG_TO_RAD);
                voltage = haptic_pid(current_attractor_angle - wrapped_angle);

                // This part keeps it compatible with your color grid display
                // It maps the 36 spinner positions to the 16 grid colors
                int spinner_position = round(current_attractor_angle / (COLOR_MODE_DETENT_ANGLE * DEG_TO_RAD));
                current_color_index = spinner_position % 16;
                break;
            }
        }
    }
    motor.move(voltage);
}

// This function still exists so your other files will compile
int motor_get_color_index(){
    return current_color_index;
}

void motor_set_mode(KnobMode new_mode, bool force_update) {
    if ((!is_transitioning && new_mode != current_mode) || force_update) {
        if (!force_update) {
            mode_angles[current_mode] = motor.shaft_angle;
            
            if (new_mode == MODE_TEMPERATURE_CONTROL) {
                float target_candidate = mode_angles[new_mode];
                float min_rad = TEMP_MIN_ANGLE * DEG_TO_RAD;
                float max_rad = TEMP_MAX_ANGLE * DEG_TO_RAD;
                if (target_candidate < min_rad || target_candidate > max_rad) {
                    transition_target_angle = min_rad;
                } else {
                    transition_target_angle = target_candidate;
                }
            } else if (new_mode == MODE_MEDIA_CONTROL) {
                transition_target_angle = (TRACK_MODE_MIN_ANGLE + TRACK_MODE_MAX_ANGLE) / 2.0f * DEG_TO_RAD;
            } else {
                transition_target_angle = mode_angles[new_mode];
            }
            
            is_transitioning = true;
        }
        
        current_mode = new_mode;
        Serial.print("Mode set to: ");
        Serial.println(mode_names[current_mode]);

        // --- FIX: Restored simple & crisp PID settings ---
        switch (current_mode) {
            case MODE_FAN_SPEED:
            case MODE_TEMPERATURE_CONTROL:
            case MODE_LIGHT_COLOR:
                // Use crisp detents for these modes
                haptic_pid.P = HAPTIC_KP_DETENT;
                haptic_pid.D = 0; // Set D to 0 for a sharper "click"
                break;
            case MODE_LIGHT_BRIGHTNESS:
            case MODE_MEDIA_VOLUME:
            case MODE_MEDIA_CONTROL:
                // Use end-stops for these modes
                haptic_pid.P = HAPTIC_KP_ENDSTOP;
                haptic_pid.D = 0; 
                break;
        }

        if (is_transitioning && !force_update) {
            haptic_pid.P = 5; 
        }
    }
}

KnobMode motor_get_mode() {
    return current_mode;
}

float motor_get_angle_radians() {
    return motor.shaft_angle;
}

float motor_get_attractor_angle_radians() {
    return current_attractor_angle;
}

const char* motor_get_mode_name() {
    return mode_names[current_mode];
}
