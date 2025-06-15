#include <Arduino.h>
#include "config.h"
#include "motor.h"
#include "display.h"
#include "network.h"

// --- Multi-Threading Setup ---

// This is the function that will run on the dedicated motor core (Core 0).
// It contains an infinite loop that calls motor_update() continuously.
void motor_task(void *pvParameters) {
    Serial.println("Motor task started on Core 0.");
    for (;;) { // Infinite loop
        motor_update();
        // A small delay is crucial to allow the task scheduler to run.
        vTaskDelay(1); 
    }
}

// --- Button Handling Variables ---
int buttonState = HIGH;
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;

void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    motor_init();
    display_init();
    network_init();

    // --- Launch the dedicated motor task ---
    // xTaskCreatePinnedToCore(
    //      function_to_run,    // The function implementing the task
    //      "Task Name",        // A name for the task (for debugging)
    //      stack_size,         // Stack size in bytes
    //      parameters,         // Pointer to parameters passed to the task
    //      priority,           // Task priority (1 is low)
    //      task_handle,        // Task handle
    //      core_id             // Core to pin the task to (0 or 1)
    // );
    xTaskCreatePinnedToCore(
        motor_task,
        "Motor Control Task",
        4096,
        NULL,
        1,
        NULL,
        0 // Pin to Core 0
    );

    Serial.println("SmartKnob Initialized! Main loop is running on Core 1.");
}

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


// This is the main loop, running on Core 1
void loop() {
    // motor_update() is NO LONGER called from here. It runs on its own core.
    check_button();
    display_update();
    network_update();
}
