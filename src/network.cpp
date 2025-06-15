#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "config.h"
#include "network.h"
#include "motor.h"

// --- Network Objects ---
WiFiClientSecure espClient;
PubSubClient client(espClient);
unsigned long lastMqttUpdate = 0;

void reconnect_mqtt() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (client.connect("SmartKnobClient", MQTT_USER, MQTT_PASSWORD)) {
            Serial.println(" [SUCCESS]");
            // Publish initial state after connecting
            network_publish_mode(); 
        } else {
            Serial.print(" [FAILED], rc=");
            // Print the reason for failure
            Serial.print(client.state());
            Serial.println(" - Retrying in 5 seconds...");
            delay(5000);
        }
    }
}

void network_init() {
    Serial.println("--- Network Initialization ---");
    Serial.print("Connecting to WiFi SSID: ");
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // Wait for WiFi to connect
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    // --- Confirmation of successful connection ---
    Serial.println("\n---------------------------------");
    Serial.println("WiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.println("---------------------------------");
    
    // Set up MQTT server and port
    espClient.setInsecure(); // Required for some brokers
    client.setServer(MQTT_SERVER, MQTT_PORT);
    
    // Initial connection attempt is now handled in the first loop
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
    // If not connected, reconnect_mqtt() will handle it.
    if (!client.connected()) {
        reconnect_mqtt();
    }
    // This is required to process incoming messages and maintain connection
    client.loop();

    if (millis() - lastMqttUpdate > MQTT_UPDATE_MS) {
        lastMqttUpdate = millis();
        
        float angleDeg = motor_get_angle_radians() * RAD_TO_DEG;
        char payload[10];

        // Only publish if connected
        if (client.connected()) {
            switch(motor_get_mode()) {
                case MODE_LIGHT_BRIGHTNESS: {
                    float displayAngleDeg = constrain(angleDeg, 0, VOLUME_MAX_ANGLE);
                    int brightness = map(displayAngleDeg, 0, VOLUME_MAX_ANGLE, 0, 255);
                    sprintf(payload, "%d", brightness);
                    client.publish(MQTT_TOPIC_LIGHT_BRIGHTNESS, payload);
                    break;
                }
                case MODE_MEDIA_VOLUME: {
                    float displayAngleDeg = constrain(angleDeg, 0, VOLUME_MAX_ANGLE);
                    // The HA automation expects a value from 0-100 for the input_number helper
                    int volume_percent = (displayAngleDeg / VOLUME_MAX_ANGLE) * 100;
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
            }
        }
    }
}
