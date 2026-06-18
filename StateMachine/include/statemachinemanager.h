#pragma once

#include "statemacine.h"
#include "GenericState.h"
#include "fsmlist.h"
#include "DataPackage/include/DataPackage.h"

#include <functional>
#include <memory>
#include <unordered_map>
#include <vector>

using StateCreator = std::function<std::shared_ptr<StateMachine>()>;
static std::unordered_map<State, StateCreator> gStateCreatorMap;

template <typename ActionType, State StateEnum>
void registerStateDirectly() {
    using StateImpl = GenericState<ActionType, StateEnum>;
    gStateCreatorMap[StateEnum] = []() -> std::shared_ptr<StateMachine> {
        return std::make_shared<StateImpl>();
    };
}

#include "StateMachine/include/state_registry_user.h"

namespace StateRegistry {
    inline void registerAllStates() {
        #define PHYBOT_REGISTER_STATE(ActionType, StateEnum) \
            registerStateDirectly<ActionType, StateEnum>();

        PHYBOT_FOR_EACH_STATE(PHYBOT_REGISTER_STATE)

        #undef PHYBOT_REGISTER_STATE
    }
}

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
