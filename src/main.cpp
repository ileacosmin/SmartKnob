#include <Arduino.h>
#include "config.h"
#include "motor.h"
#include "display.h"
#include "network.h"

// Task for dedicated motor control on Core 0
void motor_task(void *pvParameters) {
    Serial.println("Motor task started on Core 0.");
    for (;;) {
        motor_update();
        vTaskDelay(1); 
    }
}

// Button handling variables
int buttonState = HIGH;
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;

void check_button() {
    int reading = digitalRead(BUTTON_PIN);
    if (reading != lastButtonState) {
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > BUTTON_DEBOUNCE_MS) {
        if (reading != buttonState) {
            buttonState = reading;
            if (buttonState == LOW) {
                network_publish_button_press();
                KnobMode current_mode = motor_get_mode();
                current_mode = (KnobMode)((current_mode + 1) % NUM_MODES);
                motor_set_mode(current_mode);
                network_publish_mode();
            }
        }
    }
    lastButtonState = reading;
}

void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    motor_init();
    display_init();
    network_init();

    // Launch the dedicated motor task on Core 0
    xTaskCreatePinnedToCore(
        motor_task,
        "Motor Control Task",
        4096,
        NULL,
        1,
        NULL,
        0
    );

    Serial.println("SmartKnob Initialized! Main loop is running on Core 1.");
}

// Main loop, running on Core 1
void loop() {
    // motor_update() is no longer called here; it runs on its own core.
    check_button();
    display_update();
    network_update();
}
