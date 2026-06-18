#pragma once

#include "statemacine.h"
#include "fsmlist.h"
#include "DataPackage/include/DataPackage.h"

#include <iostream>
#include <memory>
#include <type_traits>

template <typename ActionType, State StateEnum>
class GenericState : public StateMachine {
public:
    GenericState() : action_ptr(nullptr) {
        setCurrentState(StateEnum);
        printCurrentState();
    }

    void start(Event event, DataPackage& data) override {
        (void)event;
        (void)data;
        if (!action_ptr) {
            action_ptr = std::make_unique<ActionType>();
        }
    }

    void run(Event event, DataPackage& data) override {
        (void)event;
        if (!action_ptr) return;

        action_ptr->GetDataFromPackage(data);
        if constexpr (StateEnum != State::ZERO) {
            action_ptr->Step();
        }
        action_ptr->SetDataToPackage(data);
    }

    void exit(Event event, DataPackage& data) override {
        (void)event;
        if constexpr (StateEnum == State::ZERO) {
            data.If_first_Zero = true;
        } else if (action_ptr) {
            action_ptr->Exit();
        }
    }

    bool CanExit(State& current, State& next, DataPackage& data) override {
        (void)current;
        (void)next;
        (void)data;
        return true;
    }

private:
    std::unique_ptr<ActionType> action_ptr;
};
