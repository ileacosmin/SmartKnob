#pragma once

// Function declarations for network operations

/**
 * @brief Initializes WiFi and connects to the MQTT broker.
 */
void network_init();

/**
 * @brief Keeps the MQTT client connected and publishes data periodically.
 * This should be called in the main loop.
 */
void network_update();

/**
 * @brief Publishes a message to the button press topic.
 */
void network_publish_button_press();

/**
 * @brief Publishes the current knob mode to the mode state topic.
 */
void network_publish_mode();
