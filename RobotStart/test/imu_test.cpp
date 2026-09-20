#include "device/ImuReaderSelector.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <thread>

#include "../MotorList/include/MotorList.hpp"
#include "../MotorList/include/realtime_controller.hpp"
// #include "DataPackage/include/DataPackage.h"
// #define DATALOG
bool dataLog(Eigen::VectorXd &v, std::ofstream &f) {
    for (int i = 0; i < v.size(); i++) {
        f << v[i] << " ";
    }
    f << std::endl;
    return true;
}

int main() {

    ActiveImuReader reader;
    if (!reader.start()) {
    std::cerr << "Failed to start IMU reader" << std::endl;
    return EXIT_FAILURE;
    }

    DataPackage package;
    package.init();
#ifdef DATALOG
    std::ofstream foutData;
    foutData.open("./datacollection_real.txt", std::ios::out);
    Eigen::VectorXd dataL = Eigen::VectorXd::Zero(500); // 73+21*3
#endif

    std::this_thread::sleep_for(std::chrono::milliseconds(100));


    auto record_start_time = std::chrono::steady_clock::now();
    auto log_start_time = std::chrono::steady_clock::now();
    
    period_info P_info;
    periodic_task_init(&P_info, package.control_period);
    
    int frame_count = 0;
    auto last_frame_time = std::chrono::steady_clock::now();
    double last_yaw = 0;
    
    while(1)
    {
        package.getIMUdata(reader);
        auto run_start_time = std::chrono::steady_clock::now();
        
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
        std::cout << "\033[2J\033[H";  // 清屏并回到顶部
        std::cout << "=== IMU State ===\n";
        std::cout << std::fixed << std::right
          << "[" << std::setw(5) << frame_count << "] "
          << "dt=" << std::setw(6) << std::setprecision(3) << frame_dt*1000 << "ms | "<<"\n";
        std::cout << std::fixed << std::setprecision(3);


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

        // 标记异常数据
        if (frame_dt > 0.015) std::cout << "[延迟] ";  // >15ms延迟
        if (data_jump) std::cout << "[跳跃] ";
        
        std::cout << std::endl;
        
        wait_rest_of_period(&P_info);
        auto record_now_time = std::chrono::steady_clock::now();
         
        std::chrono::duration<double> elapsed_record = record_now_time - run_start_time;
        // std::cout<<"elapsed_record"<<elapsed_record.count()<<std::endl;

        auto log_now_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_log = log_now_time - log_start_time;

#ifdef DATALOG
        dataL[0] = elapsed_log.count();
        dataLog(dataL, foutData);
#endif


    }


#ifdef DATALOG
      foutData.close();
#endif

    return 0;
}

