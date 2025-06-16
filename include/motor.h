#pragma once

enum KnobMode {
    MODE_LIGHT_BRIGHTNESS,
    MODE_MEDIA_VOLUME,
    MODE_FAN_SPEED,
    MODE_TEMPERATURE_CONTROL,
    NUM_MODES
};

void motor_init();
void motor_update();
float motor_get_angle_radians();
float motor_get_attractor_angle_radians();

void motor_set_mode(KnobMode new_mode, bool force_update = false);
KnobMode motor_get_mode();
const char* motor_get_mode_name();
