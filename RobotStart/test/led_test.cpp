#include <cstdint>
#include <cmath>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "LED/include/led.h"

namespace {

struct Color {
    std::uint8_t red;
    std::uint8_t green;
    std::uint8_t blue;
    std::uint8_t brightness{0x50};
};

bool readByte(const std::string& text, std::uint8_t& value) {
    std::size_t parsed = 0;
    const int number = std::stoi(text, &parsed, 0);
    if (parsed != text.size() || number < 0 || number > 255) {
        return false;
    }
    value = static_cast<std::uint8_t>(number);
    return true;
}

std::uint8_t alphaToBrightness(double alpha) {
    if (alpha < 0.0) {
        alpha = 0.0;
    } else if (alpha > 1.0) {
        alpha = 1.0;
    }
    (void)alpha;
    return 0x50;
}

bool parseRgba(const std::string& text, Color& color) {
    const std::string prefix = "rgba(";
    if (text.compare(0, prefix.size(), prefix) != 0 || text.back() != ')') {
        return false;
    }

    std::string body = text.substr(prefix.size(), text.size() - prefix.size() - 1);
    for (char& ch : body) {
        if (ch == ',') {
            ch = ' ';
        }
    }

    int red = 0;
    int green = 0;
    int blue = 0;
    double alpha = 1.0;
    std::istringstream stream(body);
    if (!(stream >> red >> green >> blue >> alpha) || red < 0 || red > 255 ||
        green < 0 || green > 255 || blue < 0 || blue > 255) {
        return false;
    }

    color.red = static_cast<std::uint8_t>(red);
    color.green = static_cast<std::uint8_t>(green);
    color.blue = static_cast<std::uint8_t>(blue);
    color.brightness = alphaToBrightness(alpha);
    return true;
}

}  // namespace

int main(int argc, char* argv[]) {
    std::vector<std::string> args(argv + 1, argv + argc);
    if (!args.empty() && args.back() == "--solid") {
        args.pop_back();
    }

    const std::map<std::string, Color> colors = {
        {"red", {0xFF, 0x00, 0x00}},
        {"green", {0x00, 0xFF, 0x00}},
        {"blue", {0x00, 0x00, 0xFF}},
        {"white", {0xFF, 0xFF, 0xFF}},
        {"yellow", {0xFF, 0xFF, 0x00}},
        {"cyan", {0x00, 0xFF, 0xFF}},
        {"purple", {0x80, 0x00, 0xFF}},
        {"lightblue", {8, 231, 222, alphaToBrightness(0.7)}},
        {"skyblue", {10, 191, 243, alphaToBrightness(0.7)}},
    };

    Color color = colors.at("skyblue");
    if (args.size() == 1) {
        const std::string color_name = args[0];
        const auto it = colors.find(color_name);
        if (it != colors.end()) {
            color = it->second;
        } else if (!parseRgba(color_name, color)) {
            std::cerr << "usage: " << argv[0]
                      << " [red|green|blue|white|yellow|cyan|purple|lightblue|skyblue]"
                      << " | rgba(r,g,b,a) | r g b [alpha] [--solid]" << std::endl;
            return 1;
        }
    } else if (args.size() == 3 || args.size() == 4) {
        if (!readByte(args[0], color.red) || !readByte(args[1], color.green) ||
            !readByte(args[2], color.blue)) {
            std::cerr << "RGB values must be integers from 0 to 255" << std::endl;
            return 1;
        }
        if (args.size() == 4) {
            color.brightness = alphaToBrightness(std::stod(args[3]));
        }
    } else if (!args.empty()) {
        std::cerr << "usage: " << argv[0]
                  << " [red|green|blue|white|yellow|cyan|purple|lightblue|skyblue]"
                  << " | rgba(r,g,b,a) | r g b [alpha] [--solid]" << std::endl;
        return 1;
    }

    LED led;
    if (!led.init()) {
        return 1;
    }
    if (!led.SetSolidColor(color.red, color.green, color.blue, color.brightness)) {
        return 1;
    }

    return 0;
}
