#pragma once

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <thread>
#include <unordered_map>

#include "StateMachine/include/fsmlist.h"
#include "DataPackage/include/DataPackage.h"

class StateMachine {
public:
    virtual ~StateMachine() = default;

    void setCurrentState(State state) {
        currentState = state;
    }

    virtual void run(Event event, DataPackage& package) = 0;
    virtual void start(Event event, DataPackage& data) = 0;
    virtual void exit(Event event, DataPackage& data) = 0;
    virtual bool CanExit(State& current, State& next, DataPackage& data) { return true; }

    void printCurrentState() const {
        std::cout << "Current State: " << static_cast<int>(currentState) << std::endl;
    }

    std::string toString(State state) {
        switch (state) {
            case State::IDLE:
                return "IDLE";
            case State::ZERO:
                return "ZERO";
            case State::RL_walk:
                return "RL_walk";
            case State::SIMPLE_EXAMPLE:
                return "SIMPLE_EXAMPLE";
            default:
                return "UNKNOWN";
        }
    }

    State currentState = State::IDLE;
};
