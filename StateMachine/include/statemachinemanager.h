#pragma once
#include "statemacine.h"
#include "GenericState.h"
#include "fsmlist.h"
#include "DataPackage/include/DataPackage.h"
#include <unordered_map>
#include <memory>
#include <vector>
#include <functional>

#include "ZeroState/include/ZeroState.h"
#include "RL_deploy_cpg/include/rl_deploy.h"




// 第一步：定义状态创建器类型（保留）
using StateCreator = std::function<std::shared_ptr<StateMachine>()>;
static std::unordered_map<State, StateCreator> gStateCreatorMap;

// 第二步：新增「直接注册」模板函数（替代原有registerState）
template <typename ActionType, State StateEnum>
void registerStateDirectly() {
    using StateImpl = GenericState<ActionType, StateEnum>;
    gStateCreatorMap[StateEnum] = []() -> std::shared_ptr<StateMachine> {
        return std::make_shared<StateImpl>();
    };
}

// 第三步：重构StateRegistry（直接调用registerStateDirectly）
namespace StateRegistry {
    inline void registerAllStates() {
        // 一键注册所有状态（无别名，直接关联ActionType和State枚举）
        registerStateDirectly<ZeroState, State::ZERO>();
        registerStateDirectly<rl_deploy_cpg, State::RL_walk>();
        // 新增状态仅需加一行：registerStateDirectly<NewActionType, State::NEW_STATE>();
    }
}

// 第四步：保留StateMachineManager类（无需修改）
class StateMachineManager {
public:
    State CurrentState{State::ZERO};
    State NextState{State::ZERO};
    
    StateMachineManager() {
        StateRegistry::registerAllStates();
        for (const auto& [state, creator] : gStateCreatorMap) {
            registerHandler(state, creator());
        }
    }

    void registerHandler(State state, std::shared_ptr<StateMachine> handler) {
        handlers[state] = handler;
    }

    void handleEvent(State state, Event event, DataPackage& data) {
        auto handler = handlers.find(state);
        if (handler != handlers.end()) {
            if (event == Event::START) {
                handler->second->start(event, data);
            } else if (event == Event::RUN) {
                handler->second->run(event, data);
            } else if (event == Event::EXIT) {
                handler->second->exit(event, data);
            } 
        } else {
            std::cout << "No handler for state: " << static_cast<int>(state) << std::endl;
        }
    }

    void init(DataPackage& data) {
        for (const auto& [state, _] : gStateCreatorMap) {
            handleEvent(state, Event::START, data);
        }
    }

    void GetDataFromPackage(DataPackage &DataPackage);
    void SetDataToPackage(DataPackage &DataPackage);
    void run(DataPackage &DataPackage);

private:
    std::unordered_map<State, std::shared_ptr<StateMachine>> handlers;
    std::unordered_map<State, State> stateTransitionMap;    
};