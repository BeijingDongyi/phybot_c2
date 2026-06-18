#pragma once

#include "DataPackage/include/DataPackage.h"

#include <iostream>

class SimpleStateExample {
public:
    SimpleStateExample() = default;

    void GetDataFromPackage(DataPackage& data) {
        counter_++;
        current_state_ = data.CurrentState;
    }

    void Step() {
        if (counter_ % 1000 == 0) {
            std::cout << "SimpleStateExample running, counter=" << counter_ << std::endl;
        }
    }

    void SetDataToPackage(DataPackage& data) {
        data.CurrentState = current_state_;
    }

    void Exit() {
        std::cout << "SimpleStateExample exit" << std::endl;
    }

private:
    int counter_{0};
    State current_state_{State::IDLE};
};
