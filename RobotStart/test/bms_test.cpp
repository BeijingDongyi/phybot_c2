#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <thread>

#include "DataPackage.h"
#include "MotorList.hpp"

int main(int argc, char* argv[]) {
    int count = 10;
    int interval_ms = 1000;

    if (argc >= 2) {
        count = std::atoi(argv[1]);
    }
    if (argc >= 3) {
        interval_ms = std::atoi(argv[2]);
    }
    if (count <= 0) {
        count = 1;
    }
    if (interval_ms < 0) {
        interval_ms = 0;
    }

    DataPackage package;
    package.init();

    const std::string config_yaml1 = "../MotorList/config/phybot_mini_1.yaml";
    const std::string config_yaml2 = "../MotorList/config/phybot_mini_2.yaml";

    MotorList motorlist;
    motorlist.Init(config_yaml1, config_yaml2, package);

    std::cout << "BMS voltage test start, count=" << count
              << ", interval_ms=" << interval_ms << std::endl;

    for (int i = 0; i < count; ++i) {
        const bool ok = motorlist.UpdateBMSInfo(package);
        if (!ok || !package.bms_valid) {
            std::cerr << "[" << i << "] BMS read failed" << std::endl;
        } else {
            std::cout << "[" << i << "] voltage="
                      << std::fixed << std::setprecision(2)
                      << package.bms_voltage << " V"
                      << ", current=" << package.bms_current << " A"
                      << ", soc=" << package.bms_soc << " %"
                      << ", status=" << static_cast<int>(package.bms_battery_status)
                      << std::endl;
        }

        if (i + 1 < count && interval_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
        }
    }

    return 0;
}
