#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <cstdlib>

#include "DataPackage.h"
#include "MotorList.hpp"

using namespace std;

int main(int argc, char* argv[]) {
    // ===================== 可修改参数 =====================
    int target_index = 0;     // 要调 PD 的电机 index，看下面打印出来的表
    float new_p = 50.0f;      // 新的 PDKp
    float new_d = 5.0f;       // 新的 PDKd

    // 也可以运行: ./pd_control 0 50 5
    if (argc >= 4) {
        target_index = atoi(argv[1]);
        new_p = static_cast<float>(atof(argv[2]));
        new_d = static_cast<float>(atof(argv[3]));
    }

    // ===================== 初始化 =====================
    DataPackage package;
    package.init();

    string yaml1 = "../MotorList/config/phybot_mini_1.yaml";
    string yaml2 = "../MotorList/config/phybot_mini_2.yaml";

    MotorList motorlist;
    motorlist.Init(yaml1, yaml2, package);
    cout << "电机数量: " << motorlist.num_motors << endl;

    if (target_index < 0 || target_index >= motorlist.num_motors) {
        cerr << "target_index 超出范围: " << target_index << endl;
        return EXIT_FAILURE;
    }

    // ===================== 按配置里的初始化 PD 下发 =====================
    cout << "index\tid\tname\tinit_PDKp\tinit_PDKd" << endl;

    int index = 0;
    for (const auto& motorPair : *motorlist.Motors_Map) {
        float init_p = static_cast<float>(motorlist.P_control_vector(index));
        float init_d = static_cast<float>(motorlist.D_control_vector(index));

        cout << index << "\t"
             << motorPair.second->Id << "\t"
             << motorPair.first << "\t"
             << fixed << setprecision(6)
             << init_p << "\t"
             << init_d << endl;

        if (!motorPair.second->SetPD(init_p, init_d)) {
            cerr << "初始化 SetPD 失败: index=" << index
                 << ", name=" << motorPair.first << endl;
            return EXIT_FAILURE;
        }

        index++;
        this_thread::sleep_for(chrono::milliseconds(5));
    }

    // ===================== 使能 =====================
    if (!motorlist.Enable()) {
        cerr << "电机使能失败，程序退出" << endl;
        return EXIT_FAILURE;
    }

    this_thread::sleep_for(chrono::milliseconds(100));

    // ===================== 对目标电机动态 Set/Get PD =====================
    index = 0;
    for (const auto& motorPair : *motorlist.Motors_Map) {
        if (index == target_index) {
            float before_p = 0.0f;
            float before_d = 0.0f;
            float after_p = 0.0f;
            float after_d = 0.0f;

            cout << "\n目标电机: index=" << index
                 << ", id=" << motorPair.second->Id
                 << ", name=" << motorPair.first << endl;

            if (!motorPair.second->GetP(before_p) || !motorPair.second->Getd(before_d)) {
                cerr << "修改前 GetP/Getd 失败" << endl;
                return EXIT_FAILURE;
            }

            cout << "before: PDKp=" << before_p
                 << ", PDKd=" << before_d << endl;

            if (!motorPair.second->SetPD(new_p, new_d)) {
                cerr << "动态 SetPD 失败" << endl;
                return EXIT_FAILURE;
            }

            this_thread::sleep_for(chrono::milliseconds(100));

            if (!motorPair.second->GetP(after_p) || !motorPair.second->Getd(after_d)) {
                cerr << "修改后 GetP/Getd 失败" << endl;
                return EXIT_FAILURE;
            }

            cout << "after : PDKp=" << after_p
                 << ", PDKd=" << after_d << endl;
            cout << "PD 动态设置完成" << endl;
            break;
        }

        index++;
    }

    return 0;
}
