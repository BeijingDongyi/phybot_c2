#include <iostream>

#include "LED/include/led.h"

int main() {
    LED led;
    if (!led.init()) {
        return 1;
    }
    if (!led.SetSolidColor(0x00, 0x00, 0x00)) {
        return 1;
    }

    return 0;
}
