/* 防止头文件重复包含 */
#pragma once

/* include */
#include "statemacine.h"
#include "GenericState.h"
#include "fsmlist.h"
#include "DataPackage/include/DataPackage.h"
#include <unordered_map>
#include <memory>
#include <vector>
#include <functional>
#include <mutex>
#include <thread>

#include "ZeroState/include/ZeroState.h"
#include "RL_deploy_climb_up/include/rl_deploy.h"
#include "RL_deploy_crane_down/include/rl_deploy.h"
#include "RL_deploy_cpg/include/rl_deploy.h"
#include "RL_deploy_mimic/include/rl_deploy.h"
#include "RL_deploy_long_motion/include/rl_deploy.h"
#include "RL_deploy_forward_punch/include/rl_deploy.h"
#include "RL_deploy_forward_kick/include/rl_deploy.h"
#include "RL_deploy_kongfu/include/rl_deploy.h"
#include "RL_deploy_taichi/include/rl_deploy.h"
#include "RL_deploy_introduct/include/rl_deploy.h"
#include "RL_deploy_dance/include/rl_deploy.h"
#include "RL_deploy_ase/include/rl_deploy.h"
#include "RL_deploy_lower_cpg/include/rl_deploy.h"

/* 状态创建器类型定义 */
using StateCreator = std::function<std::shared_ptr<StateMachine>()>;

/* State 枚举 -> 对应的创建函数 */
static std::unordered_map<State, StateCreator> gStateCreatorMap;


template <typename ActionType, State StateEnum>
void registerStateDirectly() {
    /* 推导具体状态类型 */
    using StateImpl = GenericState<ActionType, StateEnum>;   
    gStateCreatorMap[StateEnum] = []() -> std::shared_ptr<StateMachine> {
        /* 创建状态实例 */
        return std::make_shared<StateImpl>();                
    };
}

namespace StateRegistry {
    /*
     * 注册所有状态到全局映射表
     * 每个状态通过 registerStateDirectly 将 ActionType 与 State 枚举关联
     * 新增状态只需加一行 registerStateDirectly<NewActionType, State::NEW_STATE>();
     */
    inline void registerAllStates() {
        registerStateDirectly<ZeroState,                State::ZERO>();
        registerStateDirectly<rl_deploy_climb_up,       State::RL_climb_up>();
        registerStateDirectly<rl_deploy_crane_down,     State::RL_crane_down>();
        registerStateDirectly<rl_deploy_cpg,            State::RL_walk>();
        registerStateDirectly<rl_deploy_mimic,          State::RL_mimic>();
        registerStateDirectly<rl_deploy_long_motion,    State::RL_long_motion>();
        registerStateDirectly<rl_deploy_forward_punch,  State::RL_forward_punch>();
        registerStateDirectly<rl_deploy_forward_kick,   State::RL_forward_kick>();
        registerStateDirectly<rl_deploy_taichi,         State::RL_taichi>();
        registerStateDirectly<rl_deploy_kongfu,         State::RL_kongfu>();
        registerStateDirectly<rl_deploy_introduct,      State::RL_introduct>();
        registerStateDirectly<rl_deploy_dance,          State::RL_dance>();
        /*------------------------- ASE -------------------------*/
        registerStateDirectly<rl_deploy_ase,            State::RL_ASE_combinations>();
        registerStateDirectly<rl_deploy_ase_fk,         State::RL_ASE_sidekick>();
        registerStateDirectly<rl_deploy_ase_fk,         State::RL_ASE_uppercut>();
        registerStateDirectly<rl_deploy_ase_fk,         State::RL_ASE_stand>();
        /*------------------------------------------------------*/
        /* 新增状态仅需加一行 */
    }
}

class StateMachineManager {
public:
    /* 当前状态 */
    State CurrentState{State::ZERO};    
    /* 目标状态 */
    State NextState{State::ZERO};       

 
    StateMachineManager() {
        /* 注册所有状态类型 */
        StateRegistry::registerAllStates();                     
        for (const auto& [state, creator] : gStateCreatorMap) {
            /* 创建实例并注册 */
            registerHandler(state, creator());                  
        }
    }

    ~StateMachineManager() {
        if (bg_thread_.joinable()) {
            /* 等待后台线程结束 */
            bg_thread_.join();  
        }
    }

    /* 注册状态处理器 */
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

    /* 在系统启动时预加载 ZERO、攀爬、行走三个核心状态 */
    void init(DataPackage& data) {
        handleEvent(State::ZERO,                   Event::START, data);    /* 初始化零位状态 */
        handleEvent(State::RL_walk,                Event::START, data);    /* 初始化走路状态 */
        handleEvent(State::RL_climb_up,            Event::START, data);    /* 初始化爬起状态 */ 
    }

    /* 启动后台线程加载其余非关键模型 */
    void startBackgroundLoading() {
        std::call_once(bg_load_once_, [this]() {
            bg_thread_ = std::thread([this]() {
                /* 降低线程优先级，减少对主线程的干扰 */
                nice(10);

                DataPackage dummy_pkg;

                /* 后台加载的状态列表 */
                static constexpr State to_load[] = {
                    State::RL_mimic,            /* 招手 */
                    State::RL_long_motion,      /* 出拳舞 */
                    State::RL_forward_punch,    /* 打拳 */
                    State::RL_forward_kick,     /* 踢腿 */
                    State::RL_crane_down,       /* 趴下 */
                    State::RL_taichi,           /* 太极 */
                    State::RL_kongfu,           /* 功夫 */
                    State::RL_introduct,        /* 自我介绍 */
                    State::RL_dance,            /* 舞蹈 */
                    /*-----------------------------------*/
                    State::RL_ASE_combinations, /* ase-组合拳 */
                    State::RL_ASE_sidekick,     /* ase-侧踢 */
                    State::RL_ASE_uppercut,     /* ase-上勾拳 */
                    State::RL_ASE_stand,        /* ase-站立 */
                    /*-------------------------------------------------*/
                };

                for (auto state : to_load) {
                    std::cout << ">>>>> Loading state " << static_cast<int>(state) << "<<<<<" << std::endl;

                    handleEvent(state, Event::START, dummy_pkg);

                    /* 短暂休眠 50ms，避免连续加载长时间抢占 CPU */
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }

                std::cout << ">>>>> All remaining models loaded...... <<<<<" << std::endl;
            });
        });
    }


    void GetDataFromPackage(DataPackage& DataPackage);
    void SetDataToPackage(DataPackage& DataPackage);
    void run(DataPackage& DataPackage);

    /* 测试开关：禁止进入ZERO时后台加载其余模型。 */
    void setBackgroundLoadingEnabled(bool enable) { bg_loading_enabled_ = enable; }

private:
    /* 状态处理器映射表 */
    std::unordered_map<State, std::shared_ptr<StateMachine>> handlers;
    /* 状态转换映射表 */
    std::unordered_map<State, State> stateTransitionMap;
    /* 确保后台加载线程只启动一次 */
    std::once_flag bg_load_once_;      
    /* 后台加载线程 */
    std::thread bg_thread_;

    /* 是否允许后台加载（测试可关闭） */
    bool bg_loading_enabled_{true};

};
