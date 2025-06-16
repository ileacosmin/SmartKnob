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
        
        const char* modeName = motor_get_mode_name();

        // Step 1: Clear the entire virtual screen
        sprite.fillSprite(TFT_BLACK);

        // Step 2: Draw background elements first
        switch(motor_get_mode()) {
            case MODE_LIGHT_BRIGHTNESS:
            case MODE_MEDIA_VOLUME: {
                float displayAngleDeg = constrain(angleDeg, 0, VOLUME_MAX_ANGLE);
                float percent = (displayAngleDeg / VOLUME_MAX_ANGLE) * 100.0;
                int bar_height = (percent / 100.0) * sprite.height();
                
                sprite.fillRect(0, sprite.height() - bar_height, sprite.width(), bar_height, TFT_ORANGE);
                break;
            }
            case MODE_FAN_SPEED: {
                int radius = 100;
                int num_detents = 360 / (int)FAN_DETENT_ANGLE;
                for (int i = 0; i < num_detents; i++) {
                    float attractorRad = i * FAN_DETENT_ANGLE * DEG_TO_RAD;
                    int ax = centerX + cos(attractorRad) * radius;
                    int ay = centerY - sin(attractorRad) * radius;
                    sprite.fillCircle(ax, ay, 5, TFT_BLUE);
                }
                break;
            }
        }

        // Step 3: Draw all text elements on top of the background
        // Draw the mode name at the top
        sprite.setTextDatum(TC_DATUM);
        sprite.setTextSize(2);
        sprite.setTextColor(TFT_WHITE);
        sprite.drawString(modeName, centerX, 10);

        // Draw the main value in the center
        sprite.setTextDatum(MC_DATUM); 
        sprite.setTextSize(3);
        
        switch(motor_get_mode()) {
            case MODE_LIGHT_BRIGHTNESS:
            case MODE_MEDIA_VOLUME: {
                float displayAngleDeg = constrain(angleDeg, 0, VOLUME_MAX_ANGLE);
                float percent = (displayAngleDeg / VOLUME_MAX_ANGLE) * 100.0;
                sprite.drawString(String((int)percent), centerX, centerY);
                break;
            }
            case MODE_FAN_SPEED: {
                float constrainedAngle = constrain(angleDeg, 0, 270);
                int speed_level = round(constrainedAngle / FAN_DETENT_ANGLE);
                sprite.setTextColor(TFT_GREEN);
                sprite.drawString(String(speed_level), centerX, centerY);
                break;
            }
        }

        // Step 4: Draw the needle on top of everything
        int radius = 100;
        int needleX = centerX + cos(angleRad) * radius;
        int needleY = centerY - sin(angleRad) * radius;
        sprite.drawLine(centerX, centerY, needleX, needleY, TFT_RED);
        sprite.fillCircle(needleX, needleY, 3, TFT_RED);

        // Step 5: Push the final, completed image to the screen
        sprite.pushSprite(0, 0);
    }
}
