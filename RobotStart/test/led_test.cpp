#include <cstdint>
#include <iostream>
#include <map>
#include <string>

#include "LED/include/led.h"

namespace {

struct Color {
    std::uint8_t red;
    std::uint8_t green;
    std::uint8_t blue;
};

}  // namespace

int main(int argc, char* argv[]) {
    const std::map<std::string, Color> colors = {
        {"red", {0xFF, 0x00, 0x00}},
        {"green", {0x00, 0xFF, 0x00}},
        {"blue", {0x00, 0x00, 0xFF}},
        {"white", {0xFF, 0xFF, 0xFF}},
        {"yellow", {0xFF, 0xFF, 0x00}},
        {"cyan", {0x00, 0xFF, 0xFF}},
        {"purple", {0x80, 0x00, 0xFF}},
    };

    const std::string color_name = argc > 1 ? argv[1] : "blue";
    const auto it = colors.find(color_name);
    if (it == colors.end()) {
        std::cerr << "usage: " << argv[0]
                  << " [red|green|blue|white|yellow|cyan|purple]"
                  << std::endl;
        return 1;
    }

    LED led;
    if (!led.init()) {
        return 1;
    }
    if (!led.SetSolidColor(it->second.red, it->second.green, it->second.blue)) {
        return 1;
    }

    return 0;
}
