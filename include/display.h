#pragma once

void display_init();
void display_update();

// New function to get HA payload from the palette
const char* display_get_color_payload(int index);
