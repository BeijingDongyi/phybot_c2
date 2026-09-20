#ifndef PHYBOT_LED_H
#define PHYBOT_LED_H

#include <cstddef>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>

class DataPackage;

class LED {
public:
    LED() = default;
    ~LED();

    LED(const LED&) = delete;
    LED& operator=(const LED&) = delete;

    bool init();
    bool SetCommand(std::uint16_t command);
    bool SetDataToPackage(DataPackage& package, std::uint16_t command);
    bool SetSolidColor(std::uint8_t red, std::uint8_t green, std::uint8_t blue);
    std::string GetDataFromPackage(const DataPackage& package);

private:
    bool loadConfig();
    bool openSerial();
    bool writeAllLocked(const void* data, std::size_t size);
    bool sendCommandLocked(std::uint16_t command,
                           const std::vector<std::uint8_t>& data = {});
    bool readResponseLocked(std::uint16_t expected_command);
    std::string commandText(std::uint16_t command) const;

    std::string config_path_{"LED/config/led.yaml"};
    std::string device_path_{"/dev/ttyled"};
    int serial_fd_{-1};
    std::map<std::uint16_t, std::string> command_text_;
    std::map<std::string, std::uint16_t> command_value_;
    mutable std::mutex serial_mutex_;
};

#endif
