#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "config.h"
#include "network.h"
#include "motor.h"

WiFiClientSecure espClient;
PubSubClient client(espClient);
unsigned long lastMqttUpdate = 0;

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

void network_update() {
    if (!client.connected()) {
        reconnect_mqtt();
    }
    client.loop();

    if (millis() - lastMqttUpdate > MQTT_UPDATE_MS) {
        lastMqttUpdate = millis();
        
        float angleDeg = motor_get_angle_radians() * RAD_TO_DEG;
        char payload[10];

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
                     // **FIXED:** Replaced map() with the precise floating-point calculation
                     // to ensure values with .5 are sent correctly.
                     float range_degrees = TEMP_MAX_ANGLE - TEMP_MIN_ANGLE;
                     float range_celsius = TEMP_CELSIUS_MAX - TEMP_CELSIUS_MIN;
                     float temp = TEMP_CELSIUS_MIN + ((angleDeg - TEMP_MIN_ANGLE) * range_celsius / range_degrees);

                     // Round to the nearest 0.5 to match the display and haptics
                     float rounded_temp = round(temp * 2.0) / 2.0;
                     rounded_temp = constrain(rounded_temp, TEMP_CELSIUS_MIN, TEMP_CELSIUS_MAX);
                     
                     // Format the float to a string with one decimal place
                     dtostrf(rounded_temp, 4, 1, payload);
                     client.publish(MQTT_TOPIC_TEMPERATURE, payload);
                     break;
                }
            }
        }
    }
}
