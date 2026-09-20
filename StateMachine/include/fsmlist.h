#pragma once 

#include <iostream>

// 定义状态枚举
enum class State {
IDLE,
ZERO,
RL_walk,
};

// 定义事件枚举
enum class Event {
START,
RUN,
EXIT
};

