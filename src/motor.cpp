#include <SimpleFOC.h>
#include <SPI.h>
#include "config.h"
#include "motor.h"
#include "network.h"

// ─────────────────────────────────────────────────────────────────────────────
//  Motor objects and global PID
// ─────────────────────────────────────────────────────────────────────────────
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_EN);
MagneticSensorSPI sensor = MagneticSensorSPI(SENSOR_CS, 14, 0x3FFF);
PIDController haptic_pid = PIDController(0, 0, 0, 100000, POWER_SUPPLY_VOLTAGE);

// ─────────────────────────────────────────────────────────────────────────────
//  Mode‑specific limits helper
// ─────────────────────────────────────────────────────────────────────────────
struct ModeRange {
    bool bounded;      // true  → we must stay within [min,max]
    float min_deg;     // degrees
    float max_deg;     // degrees
};

static const ModeRange mode_ranges[NUM_MODES] = {
    /* MODE_LIGHT_BRIGHTNESS  */ {true,  0,                ANALOG_MODE_MAX_ANGLE},
    /* MODE_MEDIA_VOLUME      */ {true,  0,                ANALOG_MODE_MAX_ANGLE},
    /* MODE_FAN_SPEED         */ {false, 0,                0},              // 360° continuous
    /* MODE_TEMPERATURE_CTRL  */ {true,  TEMP_MIN_ANGLE,   TEMP_MAX_ANGLE},
    /* MODE_MEDIA_CONTROL     */ {true,  TRACK_MODE_MIN_ANGLE, TRACK_MODE_MAX_ANGLE},
    /* MODE_LIGHT_COLOR       */ {false, 0,                0}               // 360° spinner
};

// ─────────────────────────────────────────────────────────────────────────────
//  State
// ─────────────────────────────────────────────────────────────────────────────
KnobMode current_mode = MODE_LIGHT_BRIGHTNESS;
const char *mode_names[] = {"Brightness", "Volume", "Fan Speed", "Temp", "Media", "Color"};

static float modeMemory[NUM_MODES] = {0};     // last *virtual* angle per mode (radians)
static float modeOffset[NUM_MODES] = {0};     // physical → virtual shift for each mode
static float currentVirtualAngle   = 0;       // live virtual angle exposed to the rest of the firmware
static float current_attractor_angle = 0;     // haptic target (virtual space)

int8_t last_media_action  = 0;  // –1 = prev, 0 = none, 1 = next
int   current_color_index = 0;  // 0‥15 (palette index)

// ─────────────────────────────────────────────────────────────────────────────
//  Utilities
// ─────────────────────────────────────────────────────────────────────────────
static float deg2rad(float d) { return d * DEG_TO_RAD; }
static float rad2deg(float r) { return r * RAD_TO_DEG; }

static float findDetent(float angle, float detent_width) {
    return roundf(angle / detent_width) * detent_width;
}

// Clamp a value (in *radians*) to the legal range for a mode.
static float clamp_to_mode(KnobMode m, float rad) {
    const ModeRange &rng = mode_ranges[m];
    if (!rng.bounded) return rad;  // unbounded – nothing to do

    float min_r = deg2rad(rng.min_deg);
    float max_r = deg2rad(rng.max_deg);
    if (rad < min_r) return min_r;
    if (rad > max_r) return max_r;
    return rad;
}

// Provide a sensible initial memory if we are entering a mode for the first time
static float default_memory_for_mode(KnobMode m, float physical_rad) {
    const ModeRange &rng = mode_ranges[m];
    if (!rng.bounded) {
        // Unbounded => stay where we ar    e (no snapping)
        return physical_rad;
    }
    // Centre of the allowed sector
    return deg2rad((rng.min_deg + rng.max_deg) * 0.5f);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Setup and Loop
// ─────────────────────────────────────────────────────────────────────────────
void motor_init() {
    SPI.begin();
    sensor.init();

    motor.linkSensor(&sensor);
    motor.sensor_direction = Direction::CW;

    driver.voltage_power_supply = POWER_SUPPLY_VOLTAGE;
    driver.init();
    motor.linkDriver(&driver);

    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.controller     = MotionControlType::torque;

    motor.init();
    motor.initFOC();
    /* ───────Low pass Filter on the velocity loop─────── */
    motor.LPF_velocity.Tf = 0.02f;        
    /* ────────────────────────────────────────────────── */

    /* ───────Low pass Filter on the angle loop─────── */
    motor.LPF_angle.Tf = 0.002f;    
    /* ────────────────────────────────────────────────── */

    // Fill memory defaults so the first switch into any mode is quiet
    float physical_now = motor.shaft_angle;
    for (int i = 0; i < NUM_MODES; ++i) {
        modeMemory[i] = default_memory_for_mode((KnobMode)i, physical_now);
    }

    motor_set_mode(MODE_LIGHT_BRIGHTNESS, true);
    Serial.println("Motor initialised.");
}

void motor_update() {
    motor.loopFOC();

    // ── 1) physical → virtual conversion ────────────────────────────────────
    float physical = motor.shaft_angle;
    currentVirtualAngle = physical + modeOffset[current_mode];

    float angle_rad = currentVirtualAngle;
    float angle_deg = rad2deg(angle_rad);

    float voltage = 0.0f;

    // ── 2) per‑mode haptic behaviour ────────────────────────────────────────
    switch (current_mode) {
        // ► Analog end‑stops 0…180°
        case MODE_LIGHT_BRIGHTNESS:
        case MODE_MEDIA_VOLUME: {
            if (angle_deg > ANALOG_MODE_MAX_ANGLE) {
                current_attractor_angle = deg2rad(ANALOG_MODE_MAX_ANGLE);
                voltage = haptic_pid(current_attractor_angle - angle_rad);
            } else if (angle_deg < 0) {
                current_attractor_angle = 0;
                voltage = haptic_pid(current_attractor_angle - angle_rad);
            } else {
                voltage = -HAPTIC_KD_DAMPING * motor.shaft_velocity;
            }
            break;
        }

        // ► 4‑speed fan detents every 90° (0…359° valid)
        case MODE_FAN_SPEED: {
            current_attractor_angle = findDetent(angle_rad, deg2rad(FAN_DETENT_ANGLE));
            voltage = haptic_pid(current_attractor_angle - angle_rad);
            break;
        }

        // ► Temperature arc 45…315° with 9° (≈0.5 °C) detents
        case MODE_TEMPERATURE_CONTROL: {
            float min_r = deg2rad(TEMP_MIN_ANGLE);
            float max_r = deg2rad(TEMP_MAX_ANGLE);

            if (angle_rad > max_r) {
                current_attractor_angle = max_r;
                voltage = haptic_pid(current_attractor_angle - angle_rad);
            } else if (angle_rad < min_r) {
                current_attractor_angle = min_r;
                voltage = haptic_pid(current_attractor_angle - angle_rad);
            } else {
                float relative = angle_rad - min_r;
                float detent_w = deg2rad(TEMP_DETENT_ANGLE);
                current_attractor_angle = findDetent(relative, detent_w) + min_r;
                voltage = haptic_pid(current_attractor_angle - angle_rad);
            }
            break;
        }

        // ► Media track paddle (spring between 70°…110°)
        case MODE_MEDIA_CONTROL: {
            float min_r = deg2rad(TRACK_MODE_MIN_ANGLE);
            float max_r = deg2rad(TRACK_MODE_MAX_ANGLE);

            if (angle_rad > max_r) {
                current_attractor_angle = max_r;
                voltage = haptic_pid(current_attractor_angle - angle_rad);
                if (last_media_action != 1) {
                    network_publish_media_control("NEXT");
                    last_media_action = 1;
                }
            } else if (angle_rad < min_r) {
                current_attractor_angle = min_r;
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

        // ► 360° colour spinner (20° detents)
        case MODE_LIGHT_COLOR: {
    float angle_rad = fmodf(currentVirtualAngle, 2 * PI);
    if (angle_rad < 0) angle_rad += 2 * PI;

    float angle_deg = rad2deg(angle_rad);
    float detent_size_deg = 360.0f / COLOR_PALETTE_SIZE;
    int index = (int)(angle_deg / detent_size_deg + 0.5f) % COLOR_PALETTE_SIZE;

    current_color_index = index;

    float target_deg = index * detent_size_deg;
    float target_rad = deg2rad(target_deg);

    float angle_error = target_rad - angle_rad;
    if (angle_error > PI) angle_error -= 2 * PI;
    if (angle_error < -PI) angle_error += 2 * PI;

    voltage = haptic_pid(angle_error);
    current_attractor_angle = target_rad;

    Serial.printf("Angle: %.1f°, Color index: %d\n", angle_deg, current_color_index);

    break;
}

    }

    // ── 3) drive motor & remember position ──────────────────────────────────
    motor.move(voltage);
    modeMemory[current_mode] = currentVirtualAngle; // update memory live
}

// ─────────────────────────────────────────────────────────────────────────────
//  Mode switching (no physical movement!)
// ─────────────────────────────────────────────────────────────────────────────
void motor_set_mode(KnobMode new_mode, bool force) {
    if (!force && new_mode == current_mode) return;

    // 1) snapshot the angle we’re leaving (virtual space)
    modeMemory[current_mode] = currentVirtualAngle;

    // 2) ensure stored value for the target mode is legal
    float stored   = modeMemory[new_mode];
    float physical = motor.shaft_angle;

    // If we have never visited this mode, stored == 0 → pick a sensible default
    if (stored == 0.0f && new_mode != MODE_LIGHT_BRIGHTNESS) {
        stored = default_memory_for_mode(new_mode, physical);
    }

    stored = clamp_to_mode(new_mode, stored);
    modeMemory[new_mode] = stored;

    // 3) choose new offset so that virtual angle == stored (no torque)
    modeOffset[new_mode] = stored - physical;

    current_mode = new_mode;

    Serial.print("Mode set to: ");
    Serial.println(mode_names[current_mode]);

    // 4) quick PID profile switch
    switch (current_mode) {
        case MODE_FAN_SPEED:
            haptic_pid.P = HAPTIC_KP_FAN;
            haptic_pid.D = 0;
            break;
        case MODE_TEMPERATURE_CONTROL:
            haptic_pid.P = HAPTIC_KP_TEMP;
            haptic_pid.D = 0;
            break;
        case MODE_LIGHT_COLOR:
            haptic_pid.P = HAPTIC_KP_COLOR;
            haptic_pid.D = 0;
            break;

        case MODE_LIGHT_BRIGHTNESS:
            haptic_pid.P = HAPTIC_KP_BRIGHTNESS;
            haptic_pid.D = 0;
            break;
        case MODE_MEDIA_VOLUME:
            haptic_pid.P = HAPTIC_KP_VOLUME;
            haptic_pid.D = 0;
            break;
        case MODE_MEDIA_CONTROL:
            haptic_pid.P = HAPTIC_KP_TRACK;
            haptic_pid.D = 0;
            break;
    }

    
}



// ─────────────────────────────────────────────────────────────────────────────
//  Simple getters (all in virtual angle space)
// ─────────────────────────────────────────────────────────────────────────────
KnobMode motor_get_mode()             { return current_mode; }
float    motor_get_angle_radians()    { return currentVirtualAngle; }
float    motor_get_attractor_angle_radians() { return current_attractor_angle; }
const char *motor_get_mode_name()     { return mode_names[current_mode]; }
int      motor_get_color_index()      { return current_color_index; }
