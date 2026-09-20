#include "DataPackage/include/DataPackage.h"
#include "LED/include/led.h"

int main() {
    DataPackage package;
    package.led_command = 0x0005;

    LED led;
    if (!led.init()) {
        return 1;
    }
    if (!led.SetDataToPackage(package, package.led_command)) {
        return 1;
    }
    if (led.GetDataFromPackage(package).empty()) {
        return 1;
    }

    return 0;
}
