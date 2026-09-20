///home/gu/文档/can_open_2.3.1_7.24/can_open_2.3.1/device/Imu_hipnuc/linux/test.cpp
#include "HipnucReader.h"
#include "log.h"
#include <cstdint>
#include <ctime>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
int main() {


    HipnucReader reader;


    reader.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

while (true)
{
    std::cout << "\033[2J\033[H";  // 清屏并回到顶部
    std::cout << std::fixed << std::setprecision(3);

    std::cout << "=== IMU State ===\n";
    std::cout << "RPY:\n";
    std::cout << "  roll  = " << reader.rpy(0) << "\n";
    std::cout << "  pitch = " << reader.rpy(1) << "\n";
    std::cout << "  yaw   = " << reader.rpy(2) << "\n\n";

    std::cout << "ZYX:\n";
    std::cout << "  z = " << reader.zyx(0) << "\n";
    std::cout << "  y = " << reader.zyx(1) << "\n";
    std::cout << "  x = " << reader.zyx(2) << "\n\n";

    std::cout << "gravity_vec = " << reader.gravity_vec.transpose() << "\n\n";

    std::cout << "acc      = " << reader.acc.transpose() << "\n";
    std::cout << "acc_new  = " << reader.acc_new.transpose() << "\n";
    std::cout << "ang_vel  = " << reader.ang_vel.transpose() << "\n";
    std::cout << "ang_vel_new = " << reader.ang_vel_new.transpose() << "\n";

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

    return 0;
}