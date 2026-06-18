#pragma once

#include "ZeroState/include/ZeroState.h"
#include "RL_deploy_cpg/include/rl_deploy.h"
#include "StateMachine/examples/SimpleStateExample.h"

// Register states here. To add a new state:
// 1. Add the state enum in StateMachine/include/fsmlist.h
// 2. Include the action class header above
// 3. Add one M(ActionClass, State::YourState) line below
#define PHYBOT_FOR_EACH_STATE(M) \
    M(ZeroState, State::ZERO) \
    M(rl_deploy_cpg, State::RL_walk) \
    M(SimpleStateExample, State::SIMPLE_EXAMPLE)
