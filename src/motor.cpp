#include <SimpleFOC.h>
#include <SPI.h>
#include "config.h"
#include "motor.h"

// --- Motor Objects ---
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_EN);
MagneticSensorSPI sensor = MagneticSensorSPI(SENSOR_CS, 14, 0x3FFF);
PIDController haptic_pid = PIDController(0, 0, 0, 100000, POWER_SUPPLY_VOLTAGE);

// --- State Management Variables ---
KnobMode current_mode = MODE_LIGHT_BRIGHTNESS;
const char* mode_names[] = {"Light", "Volume", "Fan Speed"};

// **NEW:** Array to store the last known angle for each mode
float mode_angles[NUM_MODES] = {0.0f};

// **NEW:** Flag to indicate if the motor is currently moving to a new saved position
bool is_transitioning = false;
float transition_target_angle = 0.0f;


// --- Helper Function for Detent Mode ---
float findAttractor(float angle) {
    float attractor_distance = FAN_DETENT_ANGLE * DEG_TO_RAD;
    return round(angle / attractor_distance) * attractor_distance;
}

// --- Public Functions ---
void motor_init() {
    SPI.begin();
    sensor.init();
    motor.linkSensor(&sensor);
    motor.sensor_direction = Direction::CCW;
    
    driver.voltage_power_supply = POWER_SUPPLY_VOLTAGE;
    driver.init();
    motor.linkDriver(&driver);
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.controller = MotionControlType::torque;
    motor.init();
    motor.initFOC();
    motor_set_mode(MODE_LIGHT_BRIGHTNESS); 
    Serial.println("Motor Initialized.");
}

void motor_update() {
    motor.loopFOC();
    float voltage = 0;

    // **NEW LOGIC:** Check if we are transitioning to a new position first
    if (is_transitioning) {
        // Move smoothly to the target angle.
        // The P value acts as the "strength" of the move.
        voltage = haptic_pid(transition_target_angle - motor.shaft_angle);

        // If we are very close to the target, stop transitioning.
        if (abs(transition_target_angle - motor.shaft_angle) < 0.01) { // Approx 3 degrees tolerance
            is_transitioning = false;
            // Restore PID gains for the haptic feedback of the new mode
            motor_set_mode(current_mode);
        }
    } else {
        // If not transitioning, use the normal haptic feedback logic
        switch (current_mode) {
            case MODE_FAN_SPEED: {
                float attract_angle = findAttractor(motor.shaft_angle);
                voltage = haptic_pid(attract_angle - motor.shaft_angle);
                break;
            }
            
            case MODE_LIGHT_BRIGHTNESS:
            case MODE_MEDIA_VOLUME: {
                float current_angle_deg = motor.shaft_angle * RAD_TO_DEG;
                if (current_angle_deg > VOLUME_MAX_ANGLE) {
                    voltage = haptic_pid(VOLUME_MAX_ANGLE * DEG_TO_RAD - motor.shaft_angle);
                } else if (current_angle_deg < 0) {
                     voltage = haptic_pid(0 - motor.shaft_angle);
                } else {
                    voltage = -HAPTIC_KD_DAMPING * motor.shaft_velocity;
                }
                break;
            }
        }
    }
    motor.move(voltage);
}

float motor_get_angle_radians() {
    return motor.shaft_angle;
}

// **MODIFIED:** This function now handles saving and restoring state.
void motor_set_mode(KnobMode new_mode) {
    // If the mode is actually changing, save the old state and start transition
    if (new_mode != current_mode && !is_transitioning) {
        // 1. Save the current angle for the mode we are leaving
        mode_angles[current_mode] = motor.shaft_angle;

        // 2. Set the new mode
        current_mode = new_mode;
        
        // 3. Get the target angle for the new mode and start the transition
        transition_target_angle = mode_angles[current_mode];
        is_transitioning = true;
    }
    
    // This part now only sets the PID gains for the haptic feedback
    current_mode = new_mode;
    Serial.print("Mode changed to: ");
    Serial.println(mode_names[current_mode]);

    switch (current_mode) {
        case MODE_FAN_SPEED:
            haptic_pid.P = HAPTIC_KP_DETENT;
            haptic_pid.D = 0;
            break;
        case MODE_LIGHT_BRIGHTNESS:
        case MODE_MEDIA_VOLUME:
            haptic_pid.P = HAPTIC_KP_ENDSTOP;
            haptic_pid.D = 0; 
            break;
    }

    // When transitioning, we might need a different P-gain for a smooth move
    if (is_transitioning) {
        haptic_pid.P = 5; // A gentle but firm P-gain for smooth movement
    }
}

KnobMode motor_get_mode() {
    return current_mode;
}

const char* motor_get_mode_name() {
    return mode_names[current_mode];
}
