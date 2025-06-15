#include <TFT_eSPI.h>
#include "config.h"
#include "display.h"
#include "motor.h"

// --- Display Objects and Variables ---
TFT_eSPI tft = TFT_eSPI();

// **FLICKER-FREE SOLUTION:** Create a Sprite (a virtual screen buffer in RAM)
TFT_eSprite sprite = TFT_eSprite(&tft);

unsigned long lastDisplayUpdate = 0;
int centerX;
int centerY;

void display_init() {
    tft.init();
    tft.setRotation(TFT_ROTATION);
    tft.fillScreen(TFT_BLACK); // Clear the physical screen once at the beginning

    centerX = tft.width() / 2;
    centerY = tft.height() / 2;

    // **MEMORY FIX:** Set the color depth to 8-bit (256 colors) to save RAM.
    // This halves the memory required by the sprite, preventing crashes.
    sprite.setColorDepth(8);

    // Create the sprite with the same size as the screen
    sprite.createSprite(tft.width(), tft.height());

    Serial.println("Display Initialized with Flicker-Free Sprite.");
}

void display_update() {
    if (millis() - lastDisplayUpdate > DISPLAY_UPDATE_MS) {
        lastDisplayUpdate = millis();

        float angleRad = motor_get_angle_radians();
        float angleDeg = angleRad * RAD_TO_DEG;
        
        const char* modeName = motor_get_mode_name();

        // --- Step 1: Draw everything to the invisible sprite buffer ---

        // Instead of tft.fillScreen, we fill the sprite. This is fast.
        sprite.fillSprite(TFT_BLACK);

        // Draw the top mode name to the sprite
        sprite.setTextDatum(TC_DATUM);
        sprite.setTextSize(2);
        sprite.setTextColor(TFT_WHITE);
        sprite.drawString(modeName, centerX, 10);

        // Draw UI specific to the current mode to the sprite
        sprite.setTextDatum(MC_DATUM); 
        sprite.setTextSize(3);

        switch(motor_get_mode()) {
            case MODE_LIGHT_BRIGHTNESS:
            case MODE_MEDIA_VOLUME: {
                float displayAngleDeg = constrain(angleDeg, 0, VOLUME_MAX_ANGLE);
                float percent = (displayAngleDeg / VOLUME_MAX_ANGLE) * 100.0;
                int bar_height = (percent / 100.0) * sprite.height();
                
                sprite.fillRect(0, sprite.height() - bar_height, sprite.width(), bar_height, TFT_ORANGE);
                sprite.drawString(String((int)percent), centerX, centerY);
                break;
            }

            case MODE_FAN_SPEED: {
                float constrainedAngle = constrain(angleDeg, 0, 270);
                int num_detents = 360 / (int)FAN_DETENT_ANGLE;
                int speed_level = round(constrainedAngle / FAN_DETENT_ANGLE);

                sprite.setTextColor(TFT_GREEN);
                sprite.drawString(String(speed_level), centerX, centerY);

                int radius = 100;
                for (int i = 0; i < num_detents; i++) {
                    float attractorRad = i * FAN_DETENT_ANGLE * DEG_TO_RAD;
                    int ax = centerX + cos(attractorRad) * radius;
                    int ay = centerY - sin(attractorRad) * radius;
                    sprite.fillCircle(ax, ay, 5, TFT_BLUE);
                }
                break;
            }
        }

        // Draw the needle on top of everything in the sprite
        int radius = 100;
        int needleX = centerX + cos(angleRad) * radius;
        int needleY = centerY - sin(angleRad) * radius;
        sprite.drawLine(centerX, centerY, needleX, needleY, TFT_RED);
        sprite.fillCircle(needleX, needleY, 3, TFT_RED);

        // --- Step 2: Push the completed sprite to the screen in one go ---
        sprite.pushSprite(0, 0);
    }
}
