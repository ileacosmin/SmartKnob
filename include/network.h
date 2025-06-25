#pragma once

void network_init();
void network_update();
void network_publish_button_press();
void network_publish_mode();

// function declaration for media control
void network_publish_media_control(const char* action);
