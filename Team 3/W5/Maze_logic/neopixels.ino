// ~~~~~~~~~~~~~~~~~~~~~~ NEOPIXELS FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~
void clearLights() {
    // .show() writes the values to the neopixels
    pixels.clear();
    pixels.show();
}

void showForwardLights() {
    pixels.clear();
    pixels.setPixelColor(
        FRONT_LEFT,
        pixels.Color(LED_COLOR_OFF, LED_FORWARD_GREEN, LED_COLOR_OFF)
    );
    pixels.setPixelColor(
        FRONT_RIGHT,
        pixels.Color(LED_COLOR_OFF, LED_FORWARD_GREEN, LED_COLOR_OFF)
    );
    pixels.show();
}

void showBackwardLights() {
    pixels.clear();
    pixels.setPixelColor(
        BACK_LEFT,
        pixels.Color(LED_BACKWARD_RED, LED_COLOR_OFF, LED_COLOR_OFF)
    );
    pixels.setPixelColor(
        BACK_RIGHT,
        pixels.Color(LED_BACKWARD_RED, LED_COLOR_OFF, LED_COLOR_OFF)
    );
    pixels.show();
}

void showLeftLights() {
    pixels.clear();
    pixels.setPixelColor(
        FRONT_LEFT,
        pixels.Color(LED_TURN_RED, LED_TURN_GREEN, LED_TURN_BLUE)
    );
    pixels.setPixelColor(
        BACK_LEFT,
        pixels.Color(LED_TURN_RED, LED_TURN_GREEN, LED_TURN_BLUE)
    );
    pixels.show();
}

void showRightLights() {
    pixels.clear();
    pixels.setPixelColor(
        FRONT_RIGHT,
        pixels.Color(LED_TURN_RED, LED_TURN_GREEN, LED_TURN_BLUE)
    );
    pixels.setPixelColor(
        BACK_RIGHT,
        pixels.Color(LED_TURN_RED, LED_TURN_GREEN, LED_TURN_BLUE)
    );
    pixels.show();
}
