#include "Arduino.h"
extern void setup();
extern void loop();

extern "C" void app_main() {
    // Initialize Arduino layer
    initArduino();

    // Run the standard Arduino lifecycle
    setup();
    while (true) {
        loop();
        // Yield to FreeRTOS scheduler
        vTaskDelay(1);
    }
}
