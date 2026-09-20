/*
客户开发步骤：新增机器人状态机和航模手柄按键映射

一、新增机器人状态机

1. 在软件包根目录新建状态功能目录，并建立 include、src 目录；如果状态需要
   模型、配置或动作数据，再建立 model、config、data 目录。
2. 在 include 目录创建状态类头文件，在 src 目录创建实现文件。
3. 状态类需要提供 GetDataFromPackage、Step、SetDataToPackage 和 Exit 接口，
   分别用于读取公共数据、执行状态逻辑、写回控制结果和退出状态。
4. 编辑 StateMachine/include/fsmlist.h，在 State 枚举末尾追加新状态名称，
   不要调整已有状态的顺序。
5. 编辑 StateMachine/include/statemachinemanager.h，引入新状态的头文件，并在
   StateRegistry::registerAllStates() 中注册新状态类和对应的 State 枚举。
6. 编辑 StateMachine/include/statemacine.h，在 toString() 中增加新状态名称。
7. 编辑软件包根目录的 CMakeLists.txt，把新状态源文件加入 main 目标，并把
   新状态的 include 目录加入 main 的头文件搜索路径。
8. 如果测试程序需要调用新状态，也要把相同的源文件和头文件路径加入对应
   测试目标。

二、增加航模手柄按键映射

1. 编辑 Model_airplane_Joystick/include/joystick_int.h，确认目标按键的 SBUS
   通道号和 xbox_map_t 数据字段；使用新通道时，增加通道定义和数据字段。
2. 编辑 Model_airplane_Joystick/src/joystick_int.cpp，在 Serial_map_read() 中
   读取目标通道，并设置按键松开、按下或多挡开关位置对应的字段值。
3. 在 Joystick::get_state_change() 中增加按键到新 State 枚举的映射。
4. 检查映射判断顺序。多个按键同时触发时，排在前面的条件优先执行，ZERO
   等安全状态应放在普通动作状态之前。
5. 一个按键只映射一个状态，避免同一按键同时修改多个 NextState。
6. 执行 ./autoclean.sh，再执行 ./autobuild.sh，选择 test 环境和实际使用的
   IMU；编译完成后执行 ./run.sh joy_test。
7. 逐个操作航模手柄按键，确认状态与映射一致。
8. 手柄测试通过后，重新构建 realrobot_mini 或 mujoco_sim_mini，运行 main
   验证完整的状态进入、运行和退出流程。
*/

#include <iostream>
#include <stdexcept>
#include <unistd.h>  // 用于 usleep 休眠
#include "DataPackage.h"
#include "joystick_int.h"

// ===================== 主函数（核心业务逻辑，优化后） =====================
int main(int argc, char* argv[]) {
    DataPackage package;
    package.init();

    Joystick joystick;
    joystick.init();

    // 优化 1：添加主线程休眠，与手柄线程执行频率匹配（10毫秒），降低CPU占用
    const int LOOP_DELAY_US = 10 * 1000;  // 10毫秒 = 10000微秒

    std::cout << "程序启动，开始读取手柄数据（按 Ctrl+C 退出）..." << std::endl;
    double neck_yaw_xx=0.0;
    double neck_pitch_yy=0.0;
    while(1) {
        joystick.GetDataFromPackage(package);
        joystick.run();
        joystick.SetDataToPackage(package);
        // std::cout<<"get_walk_x_direction_speed"<<joystick.get_walk_x_direction_speed()<<std::endl;
        // std::cout<<"get_walk_yaw_direction_speed"<<joystick.get_walk_yaw_direction_speed()<<std::endl;
        // std::cout<<"get_stand_up_pos"<<joystick.get_stand_up_pos()<<std::endl;
        // std::cout<<"get_stand_forward_pos"<<joystick.get_stand_forward_pos()<<std::endl;

        // std::cout<<"get_stand_left_pos"<<joystick.get_stand_left_pos()<<std::endl;
        // std::cout<<"get_stand_yaw_pos"<<joystick.get_stand_yaw_pos()<<std::endl;
        // std::cout<<"get_stand_up_vel"<<joystick.get_stand_up_vel()<<std::endl;
        // std::cout<<"get_stand_forward_vel"<<joystick.get_stand_forward_vel()<<std::endl;
        // std::cout<<"get_stand_left_vel"<<joystick.get_stand_left_vel()<<std::endl;
        // std::cout<<"get_stand_yaw_vel"<<joystick.get_stand_yaw_vel()<<std::endl;
        // std::cout<<"get_walk_y_direction_speed"<<joystick.get_walk_y_direction_speed()<<std::endl;
        // std::cout<<"get_stand_yaw_vel"<<joystick.get_stand_yaw_vel()<<std::endl;

        // std::cout<<"left_y"<<joystick.left_y(1,-0.3,0.5)<<std::endl;
        // std::cout<<"right_x"<<joystick.right_x(-1,-0.5,0.3)<<std::endl;
        // std::cout<<"right_y"<<joystick.right_y(-1,-0.5,0.3)<<std::endl;
        // std::cout<<"left_x"<<joystick.left_x(1,-0.3,0.5)<<std::endl;
        // std::cout<<"***************************************************************"<<std::endl;
        // 优化 2：打印数据（可选：每10次循环打印一次，减少刷屏）
        // static int print_count = 0;
        // if (++print_count >= 10) {
        //     std::cout << package.neck_yaw_xx << "  " 
        //               << package.neck_pitch_yy << "   " 
        //               << package.js_vx_desire << "   " 
        //               << package.js_vy_desire << "   " 
        //               << package.js_OmegaZ_desire << std::endl;
        //     print_count = 0;
        // }

        // std::cout<<package.neck_yaw_xx<<" "<<package.neck_pitch_yy<<std::endl;
        // std::cout<<neck_yaw_xx<<" "<<neck_pitch_yy<<std::endl;
        // 核心优化：主线程休眠，避免CPU 100%
        usleep(LOOP_DELAY_US);
    }

    return 0;
}
