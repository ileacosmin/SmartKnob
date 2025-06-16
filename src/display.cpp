#include <TFT_eSPI.h>
#include "config.h"
#include "display.h"
#include "motor.h"

// --- Display Objects and Variables ---
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite sprite = TFT_eSprite(&tft);

unsigned long lastDisplayUpdate = 0;
int centerX;
int centerY;

void display_init() {
    tft.init();
    tft.setRotation(TFT_ROTATION);
    tft.fillScreen(TFT_BLACK); 

    centerX = tft.width() / 2;
    centerY = tft.height() / 2;

    sprite.setColorDepth(8);
    sprite.createSprite(tft.width(), tft.height());

    Serial.println("Display Initialized with Flicker-Free Sprite.");
}

void display_update() {
    if (millis() - lastDisplayUpdate > DISPLAY_UPDATE_MS) {
        lastDisplayUpdate = millis();

        float angleRad = motor_get_angle_radians();
        float angleDeg = angleRad * RAD_TO_DEG;
        float attractorRad = motor_get_attractor_angle_radians();
        float attractorDeg = attractorRad * RAD_TO_DEG;

        const char* modeName = motor_get_mode_name();
        KnobMode currentMode = motor_get_mode();

        sprite.fillSprite(TFT_BLACK);

        // --- Step 1: Draw background elements ---
        switch(currentMode) {
            case MODE_LIGHT_BRIGHTNESS:
            case MODE_MEDIA_VOLUME: {
                float displayAngleDeg = constrain(angleDeg, 0, ANALOG_MODE_MAX_ANGLE);
                float percent = (displayAngleDeg / ANALOG_MODE_MAX_ANGLE) * 100.0;
                int bar_height = (percent / 100.0) * sprite.height();
                sprite.fillRect(0, sprite.height() - bar_height, sprite.width(), bar_height, TFT_ORANGE);

                if (abs(angleRad - attractorRad) > 0.01) {
                     sprite.drawArc(centerX, centerY, 85, 82, attractorDeg - 5, attractorDeg + 5, TFT_RED, TFT_BLACK, false);
                }
                break;
            }
            case MODE_FAN_SPEED: {
                int radius = 100;
                int num_detents = 360 / (int)FAN_DETENT_ANGLE;
                for (int i = 0; i < num_detents; i++) {
                    float detentRad = i * FAN_DETENT_ANGLE * DEG_TO_RAD;
                    sprite.fillCircle(centerX + cos(detentRad) * radius, centerY - sin(detentRad) * radius, 5, TFT_BLUE);
                }
                sprite.drawArc(centerX, centerY, 102, 99, attractorDeg - 5, attractorDeg + 5, TFT_CYAN, TFT_BLACK, false);
                break;
            }
            case MODE_TEMPERATURE_CONTROL: {
                 // Draw background arc
                 sprite.drawArc(centerX, centerY, 105, 102, TEMP_MIN_ANGLE, TEMP_MAX_ANGLE, TFT_DARKGREY, TFT_BLACK, false);
                 
                 // Draw progress arc
                 float constrained_angle = constrain(angleDeg, TEMP_MIN_ANGLE, TEMP_MAX_ANGLE);
                 if (constrained_angle > TEMP_MIN_ANGLE) {
                     sprite.drawArc(centerX, centerY, 105, 102, TEMP_MIN_ANGLE, constrained_angle, TFT_CYAN, TFT_BLACK, false);
                 }

                 // Draw end-stop indicator arc
                 if (abs(angleRad - attractorRad) > 0.01 && (angleRad < (TEMP_MIN_ANGLE * DEG_TO_RAD) || angleRad > (TEMP_MAX_ANGLE * DEG_TO_RAD) )) {
                     sprite.drawArc(centerX, centerY, 105, 102, attractorDeg - 5, attractorDeg + 5, TFT_RED, TFT_BLACK, false);
                 }
                break;
            }
        }

        // --- Step 2: Draw all text elements ---
        sprite.setTextDatum(TC_DATUM);
        sprite.setTextSize(2);
        sprite.setTextColor(TFT_WHITE);
        sprite.drawString(modeName, centerX, 10);

        sprite.setTextDatum(MC_DATUM); 
        sprite.setTextSize(3);
        
        switch(currentMode) {
            case MODE_LIGHT_BRIGHTNESS:
            case MODE_MEDIA_VOLUME: {
                float displayAngleDeg = constrain(angleDeg, 0, ANALOG_MODE_MAX_ANGLE);
                float percent = (displayAngleDeg / ANALOG_MODE_MAX_ANGLE) * 100.0;
                sprite.drawString(String((int)percent), centerX, centerY);
                break;
            }
            case MODE_FAN_SPEED: {
                float constrainedAngle = constrain(angleDeg, 0, 359);
                int speed_level = round(constrainedAngle / FAN_DETENT_ANGLE);
                sprite.setTextColor(TFT_GREEN);
                sprite.drawString(String(speed_level), centerX, centerY);
                break;
            }
            case MODE_TEMPERATURE_CONTROL: {
                 float range_degrees = TEMP_MAX_ANGLE - TEMP_MIN_ANGLE;
                 float range_celsius = TEMP_CELSIUS_MAX - TEMP_CELSIUS_MIN;
                 float temp = TEMP_CELSIUS_MIN + ((angleDeg - TEMP_MIN_ANGLE) * range_celsius / range_degrees);

                 // **NEW:** Round the calculated temperature to the nearest 0.5
                 float rounded_temp = round(temp * 2.0) / 2.0;

                 rounded_temp = constrain(rounded_temp, TEMP_CELSIUS_MIN, TEMP_CELSIUS_MAX);
                 sprite.setTextColor(TFT_CYAN);
                 sprite.drawString(String(rounded_temp, 1) + " C", centerX, centerY);
                 break;
            }
        }

        // --- Step 3: Draw the needle (unless in Temperature mode) ---
        if (currentMode != MODE_TEMPERATURE_CONTROL) {
            int radius = 100;
            sprite.drawLine(centerX, centerY, centerX + cos(angleRad) * radius, centerY - sin(angleRad) * radius, TFT_RED);
            sprite.fillCircle(centerX + cos(angleRad) * radius, centerY - sin(angleRad) * radius, 3, TFT_RED);
        }

        // --- Step 4: Push to screen ---
        sprite.pushSprite(0, 0);
    }
}
