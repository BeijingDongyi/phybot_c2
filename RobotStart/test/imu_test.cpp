#include "HipnucReader.h"
#include "commands.h"
#include "global_options.h"
#include "log.h"
#include <cstdint>
#include <ctime>
#include <cstring>
#include "serial_port.h"    // 假设你有串口操作的函数
#include "hipnuc_dec.h"
#include "nmea_dec.h"
#include "hex2bin.h"
#include "kboot.h"
#include "log.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <iomanip>
#include"../MotorList/include/MotorList.hpp"
#include"../MotorList/include/realtime_controller.hpp"
// #include "DataPackage/include/DataPackage.h"

int main() {

    HipnucReader reader;
    reader.start();

    DataPackage package;
    package.init();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));


    period_info P_info;
    periodic_task_init(&P_info, package.control_period);
    
    int frame_count = 0;
    auto last_frame_time = std::chrono::steady_clock::now();
    double last_yaw = 0;
    
    while(1)
    {
        package.getIMUdata(reader);
        
        frame_count++;
        
        // 检测帧时间差（用于丢帧检测）
        auto now = std::chrono::steady_clock::now();
        auto frame_dt = std::chrono::duration<double>(now - last_frame_time).count();
        last_frame_time = now;
        
        // 检测数据跳跃（用于提帧/丢帧检测）
        // 考虑欧拉角的环绕性：±π 是同一位置
        double yaw_delta = reader.zyx(0) - last_yaw;
        // 如果差值超过 π，则取相反方向（更短的路径）
        if (yaw_delta > M_PI) yaw_delta -= 2 * M_PI;
        if (yaw_delta < -M_PI) yaw_delta += 2 * M_PI;
        
        bool data_jump = (std::abs(yaw_delta) > 0.3);  // 单帧超过 0.3 rad (~17°) 为跳跃
        last_yaw = reader.zyx(0);
        
        // 输出诊断信息（使用 fixed 和 setprecision 保持格式一致）
       std::cout << std::fixed << std::right
          << "[" << std::setw(5) << frame_count << "] "
          << "dt=" << std::setw(6) << std::setprecision(3) << frame_dt*1000 << "ms | "
          << "Yaw="   << std::setw(7) << std::setprecision(5) << reader.zyx(0) << "  "
          << "Pitch=" << std::setw(7) << std::setprecision(5) << reader.zyx(1) << "  "
          << "Roll="  << std::setw(7) << std::setprecision(5) << reader.zyx(2);
        
        // 标记异常数据
        if (frame_dt > 0.015) std::cout << "[延迟] ";  // >15ms延迟
        if (data_jump) std::cout << "[跳跃] ";
        
        std::cout << std::endl;
        
        wait_rest_of_period(&P_info);
    }

    return 0;
}
