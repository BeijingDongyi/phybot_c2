#include <iostream>
#include <Eigen/Dense>
#include "DataPackage.h"
#include "realtime_controller.hpp"
#include "MotorList.hpp"
#include <vector>
#include <string>
#include <stdexcept>
#include <iomanip>
#include <cmath>
#include <cstdlib>
#include "DataPackage/include/DataPackage.h"

using namespace std;

// ===================== 工具函数 =====================
vector<float> quintic_interpolate(const vector<float>& start,
                                  const vector<float>& end,
                                  float t) {
    if (start.size() != end.size()) {
        throw invalid_argument("起始与目标向量长度不一致");
    }

    float t3 = t * t * t;
    float t4 = t3 * t;
    float t5 = t4 * t;
    float factor = 6.0f * t5 - 15.0f * t4 + 10.0f * t3;

    vector<float> res;
    res.reserve(start.size());
    for (size_t i = 0; i < start.size(); ++i) {
        res.push_back(start[i] + factor * (end[i] - start[i]));
    }
    return res;
}

vector<float> eigen_to_vector(const Eigen::VectorXd& data) {
    vector<float> res(data.size());
    for (int i = 0; i < data.size(); ++i) {
        res[i] = static_cast<float>(data(i));
    }
    return res;
}

Eigen::VectorXd vector_to_eigen(const vector<float>& data) {
    Eigen::VectorXd res(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
        res(static_cast<int>(i)) = data[i];
    }
    return res;
}

bool check_zero_position(const vector<float>& zero_pos) {
    for (size_t i = 0; i < zero_pos.size(); ++i) {
        if (fabs(zero_pos[i]) > 1.0f) {
            cerr << "归零目标点超过限制: index=" << i
                 << ", value=" << zero_pos[i]
                 << ", 允许范围 [-1, 1]" << endl;
            return false;
        }
    }
    return true;
}

// ===================== 电机控制封装函数 =====================

// 通过 GetStatesToPackage 获取当前所有电机位置
vector<float> get_current_motor_positions(MotorList& motorlist,
                                          DataPackage& package,
                                          Eigen::VectorXd& pos_actual,
                                          Eigen::VectorXd& vel_actual,
                                          Eigen::VectorXd& tor_actual,
                                          Eigen::VectorXd& direction_vector) {
    motorlist.GetStatesToPackage(pos_actual, vel_actual, tor_actual,
                                 direction_vector, package, 0);
    return eigen_to_vector(pos_actual);
}

// 打印当前电机位置与归零目标位置，方便客户查看 21 个电机的控制顺序
void print_motor_positions(MotorList& motorlist,
                           DataPackage& package,
                           Eigen::VectorXd& pos_actual,
                           Eigen::VectorXd& vel_actual,
                           Eigen::VectorXd& tor_actual,
                           Eigen::VectorXd& direction_vector,
                           const vector<float>& cmd_pos,
                           const vector<float>& zero_pos,
                           int step) {
    vector<float> current_pos = get_current_motor_positions(motorlist, package,
                                                            pos_actual, vel_actual,
                                                            tor_actual, direction_vector);

    cout << "\nstep: " << step << endl;
    cout << "index\tid\tname\tcurrent(rad)\tcmd(rad)\tzero(rad)" << endl;

    int i = 0;
    for (const auto& pair : *motorlist.Motors_Map) {
        cout << i << "\t"
             << pair.second->Id << "\t"
             << pair.first << "\t"
             << fixed << setprecision(6)
             << current_pos[i] << "\t"
             << cmd_pos[i] << "\t"
             << zero_pos[i] << endl;
        i++;
    }
}

// 五阶插值平滑归零到指定初始位置
void move_to_zero_smooth(MotorList& motorlist,
                         DataPackage& package,
                         period_info& P_info,
                         const vector<float>& zero_pos,
                         Eigen::VectorXd& pos_actual,
                         Eigen::VectorXd& vel_actual,
                         Eigen::VectorXd& tor_actual,
                         Eigen::VectorXd& pos_desire,
                         Eigen::VectorXd& vel_desire,
                         Eigen::VectorXd& tor_desire,
                         Eigen::VectorXd& direction_vector,
                         int total_steps = 500,
                         int print_interval = 100) {
    vector<float> start_pos = get_current_motor_positions(motorlist, package,
                                                          pos_actual, vel_actual,
                                                          tor_actual, direction_vector);

    for (int step = 0; step <= total_steps; ++step) {
        float t = static_cast<float>(step) / total_steps;
        vector<float> interp_pos = quintic_interpolate(start_pos, zero_pos, t);

        package.motor_Pos_desire = vector_to_eigen(interp_pos);
        package.motor_Vel_desire = Eigen::VectorXd::Zero(motorlist.num_motors);
        package.motor_Torque_desire = Eigen::VectorXd::Zero(motorlist.num_motors);

        motorlist.SetCommandsFromPackage(pos_desire, vel_desire, tor_desire,
                                         direction_vector, package);

        if (step == 0 || step == total_steps || step % print_interval == 0) {
            print_motor_positions(motorlist, package, pos_actual, vel_actual,
                                  tor_actual, direction_vector,
                                  interp_pos, zero_pos, step);
        }

        wait_rest_of_period(&P_info);
    }
}

// ===================== 主函数 =====================
int main(int argc, char* argv[]) {
    // 初始化
    DataPackage package;
    package.init();

    string yaml1 = "../MotorList/config/phybot_mini_1.yaml";
    string yaml2 = "../MotorList/config/phybot_mini_2.yaml";

    MotorList motorlist;
    motorlist.Init(yaml1, yaml2, package);
    cout << "电机数量: " << motorlist.num_motors << endl;

    if (motorlist.num_motors != 21) {
        cerr << "phybot_mini 示例需要 21 个电机，当前读取到: "
             << motorlist.num_motors << endl;
        return EXIT_FAILURE;
    }

    Eigen::VectorXd pos_actual = Eigen::VectorXd::Zero(motorlist.num_motors);
    Eigen::VectorXd vel_actual = Eigen::VectorXd::Zero(motorlist.num_motors);
    Eigen::VectorXd tor_actual = Eigen::VectorXd::Zero(motorlist.num_motors);

    Eigen::VectorXd pos_desire = Eigen::VectorXd::Zero(motorlist.num_motors);
    Eigen::VectorXd vel_desire = Eigen::VectorXd::Zero(motorlist.num_motors);
    Eigen::VectorXd tor_desire = Eigen::VectorXd::Zero(motorlist.num_motors);
    Eigen::VectorXd direction_vector = motorlist.direction_vector;

    package.motor_Pos_desire = pos_desire;
    package.motor_Vel_desire = vel_desire;
    package.motor_Torque_desire = tor_desire;

    // 读取初始位置（稳定读取）
    vector<float> initial_pos(motorlist.num_motors);
    for (int i = 0; i < 50; ++i) {
        initial_pos = get_current_motor_positions(motorlist, package,
                                                  pos_actual, vel_actual,
                                                  tor_actual, direction_vector);
    }

    // 使能电机
    if (!motorlist.Enable()) {
        cerr << "电机使能失败，程序退出" << endl;
        return EXIT_FAILURE;
    }

    // 归零点：这里的 21 个值就是客户想要的初始位置，顺序对应打印出来的电机 index
    vector<float> zero_pos = {
                  -0.242, 0.029, 0.007, 0.49, -0.295, 0.0,
                  -0.242, -0.029, -0.007, 0.49, -0.295, 0.0,
                   0.0,
                   0.0, 0.0, 0.0, -0.17,
                   0.0, -0.0, 0.0, -0.17
    };

    if (zero_pos.size() != static_cast<size_t>(motorlist.num_motors)) {
        cerr << "归零目标点维度错误，应为 21，当前为: " << zero_pos.size() << endl;
        return EXIT_FAILURE;
    }

    if (!check_zero_position(zero_pos)) {
        return EXIT_FAILURE;
    }

    // 周期任务
    period_info P_info;
    periodic_task_init(&P_info, package.control_period);
    const int INTERP_STEPS = 500;

    // ===================== 主循环 =====================
    move_to_zero_smooth(motorlist, package, P_info, zero_pos,
                        pos_actual, vel_actual, tor_actual,
                        pos_desire, vel_desire, tor_desire,
                        direction_vector, INTERP_STEPS);
    cout << "已平滑归零到指定初始位置" << endl;

    return 0;
}
