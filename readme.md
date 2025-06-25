# SmartKnob - A Haptic Input Device for Home Assistant

This project is a custom-built smart knob designed to provide a tactile and intuitive interface for controlling various devices and services within a Home Assistant ecosystem. It features a brushless DC motor for haptic feedback, a circular TFT display for visual feedback, and connects to Home Assistant via MQTT.

## Core Features

-   **Haptic Feedback:** The knob uses a brushless DC motor to simulate different physical interactions, such as crisp detents, hard end-stops, and free-spinning zones.
-   **Visual Feedback:** A circular TFT display shows the current control mode and its value in real-time.
-   **Modal Control:** A single press of the integrated button cycles through various control modes, making it a versatile all-in-one controller.
-   **Home Assistant Integration:** Seamlessly integrates with Home Assistant using MQTT for state reporting and control commands.

## Implemented Control Modes

The knob currently supports the following six modes, which can be cycled through by pressing the knob:
The complete configuration for Home Assistant can be found in its dedicated repository:

Home Assistant Configuration Repository: ileacosmin/SmartKnob-homeassistant-cfg
### 1. Light Brightness
-   **Function:** Controls the brightness of a light.
-   **Haptics:** Smooth, free-spinning feel between 0% and 100% with hard "walls" at both ends.
-   **Display:** Shows the current brightness percentage in the center of the screen, with a vertical bar filling up as the brightness increases.

### 2. Media Volume
-   **Function:** Controls the volume of a media player (e.g., a Nest speaker).
-   **Haptics:** Identical to the Light Brightness mode, with a smooth feel and hard end-stops.
-   **Display:** Shows the current volume percentage.

### 3. Fan Speed
-   **Function:** Controls the speed of a fan in distinct steps (e.g., 0, 1, 2, 3).
-   **Haptics:** Features four crisp, physical "clicks" or detents, one for each speed level.
-   **Display:** Shows the current speed level as a number in the center.

### 4. Temperature
-   **Function:** Sets the target temperature for a thermostat.
-   **Haptics:** Operates within a defined range (e.g., 15°C to 30°C) with detents for each 0.5-degree increment and hard stops at the minimum and maximum temperatures.
-   **Display:** Shows the current target temperature with one decimal place (e.g., "21.5 C"). An arc around the edge of the screen shows the progress within the temperature range.

### 5. Media Control
-   **Function:** Skips to the next or previous media track.
-   **Haptics:** Provides a "free-spin" zone in the middle. Turning the knob to the left or right until it hits a hard "wall" triggers the "previous" or "next" track command, respectively.
-   **Display:** Shows two arrows, `<<` and `>>`. A small dot moves between them to indicate the knob's position, and an arrow flashes when a track change is triggered.

### 6. Light Color
-   **Function:** Changes the color of a light.
-   **Haptics:** Provides 36 distinct "clicks" in a full 360-degree circle, allowing for precise color selection.
-   **Display:** Shows a large, colored circle in the center that changes hue as you turn the knob. The corresponding hue value (0-360) is displayed as a number.

## Home Assistant Setup

The knob communicates with Home Assistant via an MQTT broker. The complete configuration for Home Assistant can be found in its dedicated repository:

* **Home Assistant Configuration Repository:** [ileacosmin/SmartKnob-homeassistant-cfg](https://github.com/ileacosmin/SmartKnob-homeassistant-cfg)

The integration requires two main parts in the configuration:

1.  **`configuration.yaml`**: Defines the MQTT sensors that listen for state updates from the knob (e.g., the current mode, brightness value, color hue, etc.).
2.  **`automations.yaml`**: Contains the logic that translates the knob's state changes into actions (e.g., when `sensor.smartknob_brightness_value` changes, call the `light.turn_on` service).

This setup allows for a robust and decoupled architecture where the knob and Home Assistant communicate efficiently.

## Acknowledgements

This project is heavily inspired by and based on the original open-source SmartKnob created by Scott Bezek. All credit for the foundational hardware design, firmware concepts, and the initial inspiration goes to him.

* **Original Project:** [scottbez1/smartknob on GitHub](https://github.com/scottbez1/smartknob)

---