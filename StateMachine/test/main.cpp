/*
 * main.cpp - 状态机测试程序入口
 * 用于验证 StateMachineManager 的基本功能
 */

/* 标准库 */
#include <iostream>
#include <unordered_map>
#include <memory>
#include <functional>

/* 状态机管理器头文件 */
#include "StateMachine/include/statemachinemanager.h"

/*
 * 主函数
 * 创建状态机管理器实例，进入空循环等待
 * 实际部署时，循环体内会调用 manager.run() 驱动状态机
 */
int main() {
    /* 创建状态机管理器（构造时自动注册所有状态） */
    StateMachineManager manager;

    while (1) {
        /* 空循环占位 —— 实际使用时替换为 manager.run(data) */
        /* manager.handleEvent(State::STANDING, Event::STANDING); */
    }

    return 0;
}
