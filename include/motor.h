#pragma once

// Defines the different operating modes for the knob
enum KnobMode {
    MODE_LIGHT_BRIGHTNESS,
    MODE_MEDIA_VOLUME,
    MODE_FAN_SPEED, // Renamed from "Detent"
    NUM_MODES // Helper to count the number of modes
};

// Function declarations
void motor_init();
void motor_update();
float motor_get_angle_radians();

// Functions to manage knob mode
void motor_set_mode(KnobMode new_mode);
KnobMode motor_get_mode();
const char* motor_get_mode_name();
