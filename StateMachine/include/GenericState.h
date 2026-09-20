/* 防止头文件重复包含 */
#pragma once

/* include */
#include "statemacine.h"
#include "fsmlist.h"
#include "DataPackage/include/DataPackage.h"

#include <memory>
#include <iostream>
#include <mutex>
#include <atomic>


template <typename ActionType, State StateEnum>
class GenericState : public StateMachine {
public:
    /* 构造函数 */
    GenericState() : action_ptr(nullptr) {
        setCurrentState(StateEnum);     
        printCurrentState();            
    }

    void ensureLoaded() {
        std::call_once(load_once_, [this]() {
            std::cout << "[Preload] Loading model for state "
                      << static_cast<int>(StateEnum) << std::endl;

            /* 创建动作类实例 */
            action_ptr = std::make_unique<ActionType>();

            /* 设置加载完成标志 */
            loaded_.store(true, std::memory_order_release);
        });
    }

    bool isLoaded() const override {
        return loaded_.load(std::memory_order_acquire);
    }

    
    void start(Event event, DataPackage& package) override {
        ensureLoaded();
    }

    void run(Event event, DataPackage& package) override {
        /* 确保模型已加载 */
        ensureLoaded();

        action_ptr->GetDataFromPackage(package);

        /* Zero 状态特殊处理 */
        if constexpr (StateEnum != State::ZERO) {
            action_ptr->Step();
        }
        
        action_ptr->SetDataToPackage(package);     
    }

    void exit(Event event, DataPackage& package) override {
        /* Zero 状态特殊处理：设置首次归零标志 */
        if constexpr (StateEnum == State::ZERO) {
            package.If_first_Zero = true;
        }
        /* 非 Zero 状态：如果动作类有 Exit 方法则调用 */
        else if (action_ptr) {
            /* 编译期检查 ActionType 是否有 Exit 方法 */
            if constexpr (std::is_member_function_pointer_v<decltype(&ActionType::Exit)>) {
                action_ptr->Exit();

                /* 只执行一次 —— 清零控制消息 */
                action_ptr->SetDataToPackage(package);
            }
        }
    }

private:
    std::once_flag load_once_;                 
    std::atomic<bool> loaded_{false};          
    std::unique_ptr<ActionType> action_ptr;    
};
