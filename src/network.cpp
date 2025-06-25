#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "config.h"
#include "network.h"
#include "motor.h"
#include "display.h" // Include display.h to get color payload

WiFiClientSecure espClient;
PubSubClient client(espClient);
unsigned long lastMqttUpdate = 0;
int last_published_color_index = -1; // Track last published color to avoid spam

void reconnect_mqtt() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect("SmartKnobClient", MQTT_USER, MQTT_PASSWORD)) {
            Serial.println("connected");
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void network_init() {
    Serial.print("Connecting to ");
    Serial.println(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected.");
    
    espClient.setInsecure();
    client.setServer(MQTT_SERVER, MQTT_PORT);
    Serial.println("Networking Initialized.");
}

void network_publish_button_press() {
    if (client.connected()) {
        client.publish(MQTT_TOPIC_BUTTON_PRESS, "PRESS");
    }
}

void network_publish_mode() {
    if (client.connected()) {
        client.publish(MQTT_TOPIC_MODE_STATE, motor_get_mode_name());
    }
}

void network_publish_media_control(const char* action) {
    if (client.connected()) {
        client.publish(MQTT_TOPIC_MEDIA_CONTROL, action);
    }
}

void network_update() {
    if (!client.connected()) {
        reconnect_mqtt();
    }
    client.loop();

    if (millis() - lastMqttUpdate > MQTT_UPDATE_MS) {
        lastMqttUpdate = millis();
        
        float angleDeg = motor_get_angle_radians() * RAD_TO_DEG;
        char payload[20]; // Increased size for color payload

        if (client.connected()) {
            switch(motor_get_mode()) {
                case MODE_LIGHT_BRIGHTNESS: {
                    float displayAngleDeg = constrain(angleDeg, 0, ANALOG_MODE_MAX_ANGLE);
                    int brightness = map(displayAngleDeg, 0, ANALOG_MODE_MAX_ANGLE, 0, 255);
                    sprintf(payload, "%d", brightness);
                    client.publish(MQTT_TOPIC_LIGHT_BRIGHTNESS, payload);
                    break;
                }
                case MODE_MEDIA_VOLUME: {
                    float displayAngleDeg = constrain(angleDeg, 0, ANALOG_MODE_MAX_ANGLE);
                    int volume_percent = (displayAngleDeg / ANALOG_MODE_MAX_ANGLE) * 100;
                    sprintf(payload, "%d", volume_percent);
                    client.publish(MQTT_TOPIC_MEDIA_VOLUME, payload);
                    break;
                }
                case MODE_FAN_SPEED: {
                     float constrainedAngle = constrain(angleDeg, 0, 270);
                     int speed_level = round(constrainedAngle / FAN_DETENT_ANGLE);
                     sprintf(payload, "%d", speed_level);
                     client.publish(MQTT_TOPIC_FAN_SPEED, payload);
                     break;
                }
                case MODE_TEMPERATURE_CONTROL: {
                     float range_degrees = TEMP_MAX_ANGLE - TEMP_MIN_ANGLE;
                     float range_celsius = TEMP_CELSIUS_MAX - TEMP_CELSIUS_MIN;
                     float temp = TEMP_CELSIUS_MIN + ((angleDeg - TEMP_MIN_ANGLE) * range_celsius / range_degrees);
                     float rounded_temp = round(temp * 2.0) / 2.0;
                     rounded_temp = constrain(rounded_temp, TEMP_CELSIUS_MIN, TEMP_CELSIUS_MAX);
                     dtostrf(rounded_temp, 4, 1, payload);
                     client.publish(MQTT_TOPIC_TEMPERATURE, payload);
                     break;
                }
                // --- NEW MQTT PUBLISH LOGIC for Color Palette ---
                case MODE_LIGHT_COLOR: {
                    int current_index = motor_get_color_index();
                    // Only publish if the color has changed
                    if (current_index != last_published_color_index) {
                        const char* hs_payload = display_get_color_payload(current_index);
                        client.publish(MQTT_TOPIC_LIGHT_COLOR_HS, hs_payload);
                        last_published_color_index = current_index;
                        Serial.print("Published Color: ");
                        Serial.println(hs_payload);
                    }
                    break;
                }
                case MODE_MEDIA_CONTROL:
                    // Publishing is handled directly in motor.cpp
                    break;
            }
        }
    }
}
