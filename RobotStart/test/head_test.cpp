#include <chrono>
#include <iostream>
#include <thread>
#include <stdexcept>
#include <algorithm>  // 用于 std::clamp
#include "DataPackage.h"
#include "joystick_int.h"
#include "realtime_controller.hpp"
#include "MotorList.hpp"



// ===================== 主函数（核心业务逻辑） =====================
int main(int argc, char* argv[]) {
    try {
        // 1. 初始化核心组件
        DataPackage package;
        package.init();

        Joystick joystick;
        joystick.init();

        MotorList motorlist;
        motorlist.Init("../MotorList/config/phybot_mini_1.yaml",
                       "../MotorList/config/phybot_mini_2.yaml",
                       package);

        const int motorNum = motorlist.num_motors;
        std::cout << "Total motor count: " << motorNum << std::endl;

        Eigen::VectorXd pos_actual = Eigen::VectorXd::Zero(motorNum);
        Eigen::VectorXd vel_actual = Eigen::VectorXd::Zero(motorNum);
        Eigen::VectorXd tor_actual = Eigen::VectorXd::Zero(motorNum);

        Eigen::VectorXd pos_desire = Eigen::VectorXd::Zero(motorNum);
        Eigen::VectorXd vel_desire = Eigen::VectorXd::Zero(motorNum);
        Eigen::VectorXd tor_desire = Eigen::VectorXd::Zero(motorNum);
        Eigen::VectorXd direction_vector = Eigen::VectorXd::Zero(motorNum);
    
        direction_vector = motorlist.direction_vector;
        // 保存数据的容器（使用 vector 存储每次循环的数据）
        std::vector<std::vector<double>> pos_actual_data;
        std::vector<std::vector<double>> vel_actual_data;
        std::vector<std::vector<double>> pos_desire_data;
        std::vector<std::vector<double>> vel_desire_data;
        std::vector<std::vector<double>> tor_actual_data;
        std::vector<std::vector<double>> tor_desire_data;
           for (int i = 0; i < 50; ++i) {
        motorlist.GetStatesToPackage(pos_actual, vel_actual, tor_actual, direction_vector, package, 0);
        int motor_idx = 0;
        std::cout << "**************@@@@@@@@@@@@@@@@@@@@@" << std::endl;
        for (const auto& motorPair : *motorlist.Motors_Map) {
            std::cout << motorPair.first << "\t\t" 
                    << pos_actual(motor_idx) << "\t\t"
                    << vel_actual(motor_idx) << "\t\t"
                    << tor_actual(motor_idx) << std::endl;
            motor_idx++;
        }

    }
        // 3. 电机使能
        if (!motorlist.Enable()) {
            std::cerr << "Motor enable failed, exiting program..." << std::endl;
            return EXIT_FAILURE;
        }
        std::cout << "Head initialization done, waiting 5s..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        period_info P_info;
        periodic_task_init(&P_info, package.control_period);
        while (true) {
            joystick.GetDataFromPackage(package);
            joystick.run();
            joystick.SetDataToPackage(package);
            motorlist.GetStatesToPackage(pos_actual, vel_actual, tor_actual, direction_vector, package, 0);
            motorlist.SetCommandsFromPackage(pos_desire, vel_desire, tor_desire, direction_vector, package);
            wait_rest_of_period(&P_info);
        }
    } catch (const std::exception& e) {
        std::cerr << "Program exception: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return 0;
}
