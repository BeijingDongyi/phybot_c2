#include <iostream>
#include <Eigen/Dense>
#include "DataPackage.h"
#include "realtime_controller.hpp"
#include "MotorList.hpp"

Eigen::VectorXd g_head_pos_actual = Eigen::VectorXd::Zero(2);

int main(int argc, char* argv[]) {
    // 初始化数据包
    DataPackage package;
    package.init();
    
    std::string config_yaml1 = "../MotorList/config/phybot_mini_1.yaml";
    std::string config_yaml2 = "../MotorList/config/phybot_mini_2.yaml";
    
    MotorList motorlist;
    motorlist.Init(config_yaml1, config_yaml2, package);
    std::cout << "motorNum: " << motorlist.num_motors << std::endl;

    // 读取初始位置
    for (int i = 0; i < 50; ++i) {
        int head_idx = 0;
        for (const auto& [motorId, motor] : *motorlist.Motors_Map2) {
            float Pos, Vel, Cur, Tor;
           uint32_t FastStateMechine,  FastMosTemperature, FastWindingTemperature,FastBusVoltage;
        uint32_t FastErrorCode;

        motor->GetPVCTFast(Pos, Vel,Cur ,Tor,FastStateMechine,FastBusVoltage,FastErrorCode);
            
            if (head_idx < 2) {
                g_head_pos_actual[head_idx] = Pos;
            }
            
            std::cout << motorId << "\t" << Pos << "\t" << Vel << "\t" << Cur << "\t" << Tor << std::endl;
            head_idx++;
        }
    }
    
    // 关闭所有电机伺服
    bool init_success = true;
    for (const auto& [motorId, motor] : *motorlist.Motors_Map2) {
        if (!motor->SetControlWord(CTRL_SERVO_OFF)) {
            std::cerr << "Warning: 电机 " << motorId << " CTRL_SERVO_OFF 执行失败" << std::endl;
            init_success = false;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 初始化周期任务
    period_info P_info;
    periodic_task_init(&P_info, package.control_period);
    // 主控制循环
    if (init_success) {
        while (true) {
            // 设置电机参数
            int head_idx = 0;
            for (const auto& [motorId, motor] : *motorlist.Motors_Map2) {
                motor->SetBigparam(g_head_pos_actual[head_idx], 0, 0);
                head_idx++;
            }
            
            motorlist.MotorControl->GetMotorNet2()->setPlanningPose();
            
            // 读取并打印电机状态
            for (const auto& [motorId, motor] : *motorlist.Motors_Map2) {
                if(motorId=="01_BAN_2_CAN_1_2")
                {
                    float Pos, Vel, Cur, Tor;
                    uint32_t FastStateMechine,  FastMosTemperature, FastWindingTemperature,FastBusVoltage;
                    uint32_t FastErrorCode;

                    motor->GetPVCTFast(Pos, Vel,Cur ,Tor,FastStateMechine,FastBusVoltage,FastErrorCode);
                    std::cout<<"motorId: "<<Pos<<std::endl;
                }
            }
            std::cout << std::endl;
            
            // 等待周期结束
            wait_rest_of_period(&P_info);
        }
    } else {
        std::cerr << "初始化失败，程序终止" << std::endl;
        return 1;
    }

    return 0;
}
