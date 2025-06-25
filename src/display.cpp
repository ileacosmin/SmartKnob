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

// --- Color Palette Definition ---
struct ColorPaletteEntry {
    uint16_t tft_color;
    const char* ha_payload; // "hue,saturation"
};

ColorPaletteEntry color_palette[COLOR_PALETTE_SIZE] = {
    {TFT_WHITE,   "0,0"},    {TFT_RED,     "0,100"},  {TFT_ORANGE,  "30,100"}, {TFT_YELLOW,  "60,100"},
    {0x7FF0,      "90,100"}, {TFT_GREEN,   "120,100"},{TFT_CYAN,    "180,100"},{TFT_NAVY,    "240,100"},
    {TFT_BLUE,    "240,70"}, {TFT_PURPLE,  "270,100"},{TFT_MAGENTA, "300,100"},{TFT_PINK,    "330,100"},
    {0xFDE7,      "34,59"},  {0x867D,      "197,71"}, {0x97F3,      "120,93"}, {0xFDBB,      "351,100"}
};

// Getter function for the network component
const char* display_get_color_payload(int index) {
    if (index >= 0 && index < COLOR_PALETTE_SIZE) {
        return color_palette[index].ha_payload;
    }
    return "0,0";
}

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
        KnobMode currentMode = motor_get_mode();

        sprite.fillSprite(TFT_BLACK);

        // --- Step 1: Draw background graphics ---
        switch(currentMode) {
            case MODE_LIGHT_BRIGHTNESS:
            case MODE_MEDIA_VOLUME: {
                float displayAngleDeg = constrain(angleDeg, 0, ANALOG_MODE_MAX_ANGLE);
                float percent = (displayAngleDeg / ANALOG_MODE_MAX_ANGLE) * 100.0;
                int bar_height = (percent / 100.0) * sprite.height();
                sprite.fillRect(0, sprite.height() - bar_height, sprite.width(), bar_height, TFT_ORANGE);
                break;
            }
            case MODE_FAN_SPEED: {
                int radius = 100;
                for (int i = 0; i < 4; i++) { // 360 / 90 = 4 detents
                    float detentRad = i * FAN_DETENT_ANGLE * DEG_TO_RAD;
                    sprite.fillCircle(centerX + cos(detentRad) * radius, centerY - sin(detentRad) * radius, 5, TFT_BLUE);
                }
                break;
            }
            case MODE_TEMPERATURE_CONTROL: {
                 sprite.drawArc(centerX, centerY, 105, 102, TEMP_MIN_ANGLE, TEMP_MAX_ANGLE, TFT_DARKGREY, TFT_BLACK, false);
                 float constrained_angle = constrain(angleDeg, TEMP_MIN_ANGLE, TEMP_MAX_ANGLE);
                 if (constrained_angle > TEMP_MIN_ANGLE) {
                     sprite.drawArc(centerX, centerY, 105, 102, TEMP_MIN_ANGLE, constrained_angle, TFT_CYAN, TFT_BLACK, false);
                 }
                break;
            }
            case MODE_MEDIA_CONTROL: {
                sprite.drawTriangle(centerX - 50, centerY, centerX - 70, centerY - 15, centerX - 70, centerY + 15, TFT_DARKGREY);
                sprite.drawTriangle(centerX + 50, centerY, centerX + 70, centerY - 15, centerX + 70, centerY + 15, TFT_DARKGREY);
                if (angleDeg <= TRACK_MODE_MIN_ANGLE) {
                    sprite.fillTriangle(centerX - 50, centerY, centerX - 70, centerY - 15, centerX - 70, centerY + 15, TFT_CYAN);
                }
                if (angleDeg >= TRACK_MODE_MAX_ANGLE) {
                    sprite.fillTriangle(centerX + 50, centerY, centerX + 70, centerY - 15, centerX + 70, centerY + 15, TFT_CYAN);
                }
                float needle_pos_x = map(angleDeg, TRACK_MODE_MIN_ANGLE, TRACK_MODE_MAX_ANGLE, centerX - 30, centerX + 30);
                needle_pos_x = constrain(needle_pos_x, centerX - 30, centerX + 30);
                sprite.fillCircle(needle_pos_x, centerY, 5, TFT_WHITE);
                break;
            }
            case MODE_LIGHT_COLOR: {
                int selected_index = motor_get_color_index();
                int cols = 4, rows = 4, box_size = 40, padding = 10;
                int start_x = centerX - (cols * (box_size + padding) - padding) / 2;
                int start_y = centerY - (rows * (box_size + padding) - padding) / 2;
                for (int i = 0; i < COLOR_PALETTE_SIZE; i++) {
                    int col = i % cols, row = i / cols;
                    int box_x = start_x + col * (box_size + padding), box_y = start_y + row * (box_size + padding);
                    sprite.fillRect(box_x, box_y, box_size, box_size, color_palette[i].tft_color);
                    if (i == selected_index) {
                        sprite.drawRect(box_x - 2, box_y - 2, box_size + 4, box_size + 4, TFT_WHITE);
                    }
                }
                break;
            }
        }

        // --- Step 2: Draw Mode Name (Top Text) ---
        sprite.setTextDatum(TC_DATUM);
        sprite.setTextSize(2);
        sprite.setTextColor(TFT_WHITE);
        sprite.drawString(modeName, centerX, 10);

        // --- Step 3: Draw Value (Center Text) ---
        sprite.setTextDatum(MC_DATUM); 
        sprite.setTextSize(3);
        switch(currentMode) {
            case MODE_LIGHT_BRIGHTNESS:
            case MODE_MEDIA_VOLUME: {
                float displayAngleDeg = constrain(angleDeg, 0, ANALOG_MODE_MAX_ANGLE);
                float percent = (displayAngleDeg / ANALOG_MODE_MAX_ANGLE) * 100.0;
                sprite.drawString(String((int)percent) + "%", centerX, centerY);
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
                 float rounded_temp = round(temp * 2.0) / 2.0;
                 rounded_temp = constrain(rounded_temp, TEMP_CELSIUS_MIN, TEMP_CELSIUS_MAX);
                 sprite.setTextColor(TFT_CYAN);
                 sprite.drawString(String(rounded_temp, 1) + " C", centerX, centerY);
                 break;
            }
            case MODE_LIGHT_COLOR:
            case MODE_MEDIA_CONTROL:
                break;
        }

        // --- Step 4: Draw Needle - FIX ---
        // This condition now correctly includes Brightness, Volume, and Fan modes.
        // It will draw a needle that follows the live motor position.
        if (currentMode != MODE_TEMPERATURE_CONTROL && currentMode != MODE_LIGHT_COLOR && currentMode != MODE_MEDIA_CONTROL) {
            int radius = 100;
            sprite.drawLine(centerX, centerY, centerX + cos(angleRad) * radius, centerY - sin(angleRad) * radius, TFT_RED);
            sprite.fillCircle(centerX + cos(angleRad) * radius, centerY - sin(angleRad) * radius, 3, TFT_RED);
        }
        
        // --- Step 5: Push to screen ---
        sprite.pushSprite(0, 0);
    }
}
