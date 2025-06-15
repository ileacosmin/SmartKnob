#include <SimpleFOC.h>
#include <SPI.h>
#include "config.h"
#include "motor.h"

// --- Motor Objects ---
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_EN);
MagneticSensorSPI sensor = MagneticSensorSPI(SENSOR_CS, 14, 0x3FFF);
PIDController haptic_pid = PIDController(0, 0, 0, 100000, POWER_SUPPLY_VOLTAGE);

// --- State Variable ---
KnobMode current_mode = MODE_LIGHT_BRIGHTNESS;
const char* mode_names[] = {"Light Brightness", "Media Volume", "Fan Speed"};

// --- Helper Function for Detent Mode ---
float findAttractor(float angle) {
    float attractor_distance = FAN_DETENT_ANGLE * DEG_TO_RAD;
    return round(angle / attractor_distance) * attractor_distance;
}

// --- Public Functions ---
void motor_init() {
    // Initialize SPI for the sensor
    SPI.begin();

    sensor.init();
    motor.linkSensor(&sensor);
    
    // **DIRECTION FIX:** Tell the FOC algorithm to treat the sensor's
    // direction as Counter-Clockwise (CCW). This inverts the movement
    // to match the physical rotation.
    motor.sensor_direction = Direction::CCW;

    driver.voltage_power_supply = POWER_SUPPLY_VOLTAGE;
    driver.init();
    motor.linkDriver(&driver);
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.controller = MotionControlType::torque;
    motor.init();
    motor.initFOC();
    motor_set_mode(MODE_LIGHT_BRIGHTNESS); // Set initial mode
    Serial.println("Motor Initialized.");
}

void motor_update() {
    motor.loopFOC();
    float voltage = 0;

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
    motor.move(voltage);
}

float motor_get_angle_radians() {
    return motor.shaft_angle;
}

void motor_set_mode(KnobMode new_mode) {
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
}

KnobMode motor_get_mode() {
    return current_mode;
}

const char* motor_get_mode_name() {
    return mode_names[current_mode];
}
