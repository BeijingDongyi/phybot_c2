#pragma once
// 1. 先包含基础依赖（原有顺序保留）
#include "statemacine.h"
#include "fsmlist.h"
#include "DataPackage/include/DataPackage.h"
// 2. 提前包含所有动作类头文件（移到顶部）

#include <memory>
#include <iostream>



// 通用状态模板：适配所有状态（统一逻辑，无差异化）
template <typename ActionType, State StateEnum>
class GenericState : public StateMachine {
public:
    GenericState() : action_ptr(nullptr) {
        setCurrentState(StateEnum); // 设置当前状态枚举
        printCurrentState();
    }

    // 统一的start逻辑：初始化动作类实例
    void start(Event event, DataPackage& data) override {
        if (!action_ptr) {
            action_ptr = std::make_unique<ActionType>();
        }
    }

    // 统一的run逻辑：GetData → Step → SetData
    void run(Event event, DataPackage& data) override {
        if (!action_ptr) return;
        
        action_ptr->GetDataFromPackage(data);
        // Zero状态特殊处理：无Step（唯一差异化点）
        if constexpr (StateEnum != State::ZERO) {
            action_ptr->Step();
        }
        action_ptr->SetDataToPackage(data);
    }

    // 统一的exit逻辑
    void exit(Event event, DataPackage& data) override {
        // Zero状态特殊处理：设置If_first_Zero
        if constexpr (StateEnum == State::ZERO) {
            data.If_first_Zero = true;
        }
        // 非Zero状态：如果有Exit方法则执行（兼容所有RL状态）
        else if (action_ptr) {
            // 检查ActionType是否有Exit方法（SFINAE兼容）
            if constexpr (std::is_member_function_pointer_v<decltype(&ActionType::Exit)>) {
                action_ptr->Exit();
            }
        }
    }

    // 统一的CanExit逻辑：所有状态默认允许退出
    bool CanExit(State& current, State& next, DataPackage& data) override {
        // RL_walk特殊：CanExit直接返回true（无Exit）；其他RL状态在exit中执行Exit
        return true;
    }

private:
    std::unique_ptr<ActionType> action_ptr; // 动作类实例
};

// // 定义所有状态的类型别名（一键注册）
// using ZeroStateImpl = GenericState<ZeroState, State::ZERO>;
// using RLWalkState = GenericState<rl_deploy_cpg, State::RL_walk>;
// using RLMimicState = GenericState<rl_deploy_mimic, State::RL_mimic>;
// using RLLongMotionState = GenericState<rl_deploy_long_motion, State::RL_long_motion>;
// using RLForwardPunchState = GenericState<rl_deploy_forward_punch, State::RL_forward_punch>;
// using RLForwardKickState = GenericState<rl_deploy_forward_kick, State::RL_forward_kick>;