#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <vector>

#include "DataPackage.h"
#include "joystick_int.h"

using namespace std;

string state_to_string(State state) {
    switch (state) {
        case State::IDLE:
            return "IDLE";
        case State::ZERO:
            return "ZERO";
        case State::RL_walk:
            return "RL_walk";
        case State::SIMPLE_EXAMPLE:
            return "SIMPLE_EXAMPLE";
        default: {
            ostringstream oss;
            oss << "UNKNOWN(" << static_cast<int>(state) << ")";
            return oss.str();
        }
    }
}

string join_buttons(const vector<string>& buttons) {
    if (buttons.empty()) {
        return "none";
    }

    string text;
    for (size_t i = 0; i < buttons.size(); ++i) {
        if (i > 0) {
            text += ",";
        }
        text += buttons[i];
    }
    return text;
}

// 客户接入自定义状态的示例：
// 1. 在 StateMachine/include/fsmlist.h 里添加状态
// 2. 在 StateMachine/include/state_registry_user.h 里注册状态
// 3. 在 joystick.run() 和 joystick.SetDataToPackage(package) 之间设置 NextState
void apply_custom_state_switch(Joystick& joystick) {
    xbox_map_t button_state = joystick.get_button_state();

    if (button_state.a == 1) {
        joystick.set_next_state(State::SIMPLE_EXAMPLE);
    }
}

// ===================== 主函数 =====================
int main(int argc, char* argv[]) {
    DataPackage package;
    package.init();

    Joystick joystick;
    joystick.init();

    const int LOOP_DELAY_US = 10 * 1000;
    const int PRINT_INTERVAL = 10;
    int print_count = 0;

    cout << "程序启动，开始读取手柄数据（按 Ctrl+C 退出）..." << endl;
    cout << "默认闭源手柄映射: X -> ZERO, B -> RL_walk, A -> SIMPLE_EXAMPLE" << endl;
    cout << "客户扩展示例: 可在 apply_custom_state_switch() 里接入自己的按键和状态" << endl;
    cout << "自定义状态切换应放在 joystick.run() 和 joystick.SetDataToPackage(package) 之间" << endl;

    while (true) {
        joystick.GetDataFromPackage(package);
        joystick.run();
        apply_custom_state_switch(joystick);
        joystick.SetDataToPackage(package);

        if (++print_count >= PRINT_INTERVAL) {
            xbox_map_t button_state = joystick.get_button_state();
            vector<string> pressed_buttons = joystick.get_pressed_buttons();

            cout << "pressed=[" << join_buttons(pressed_buttons) << "]"
                 << " next_state=" << state_to_string(joystick.get_next_state())
                 << " next_state_name=" << joystick.get_next_state_name()
                 << " package.NextState=" << state_to_string(package.NextState)
                 << " control_mode=" << package.control_mode
                 << " lx=" << button_state.lx
                 << " ly=" << button_state.ly
                 << " rx=" << button_state.rx
                 << " ry=" << button_state.ry
                 << " e=" << button_state.e
                 << " f=" << button_state.f
                 << " g=" << button_state.g
                 << " h=" << button_state.h
                 << " vx=" << package.js_vx_desire
                 << " vy=" << package.js_vy_desire
                 << " yaw=" << package.js_OmegaZ_desire
                 << endl;

            print_count = 0;
        }

        usleep(LOOP_DELAY_US);
    }

    return 0;
}
