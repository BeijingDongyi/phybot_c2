/* 防止头文件重复包含 */
#pragma once

/* include */
#include <iostream>
#include <unordered_map>
#include <memory>
#include <functional>
#include <thread>
#include <chrono>
#include "StateMachine/include/fsmlist.h"
#include "DataPackage/include/DataPackage.h"



class StateMachine {
public:
    /* 虚析构函数 */
    virtual ~StateMachine() = default;

    virtual void start(Event event, DataPackage& package) = 0;
    virtual void run(Event event, DataPackage& package) = 0;
    virtual void exit(Event event, DataPackage& package) = 0;
    
    virtual bool isLoaded() const { 
        return true; 
    }

    /* 设置当前状态枚举值 */
    void setCurrentState(State state) {
        currentState = state;
    }

    /* 打印当前状态信息 */
    void printCurrentState() const {
        std::cout << "Current State: " << static_cast<int>(currentState) << std::endl;
    }

private:
    /* 当前状态枚举，默认为 IDLE */
    State currentState = State::IDLE;

};
