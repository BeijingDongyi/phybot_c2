# Simple State Registration

`SimpleStateExample` is already registered as `State::SIMPLE_EXAMPLE` in this
package. It is intentionally small and can be used as a template for customer
states.

Registered states are declared in `StateMachine/include/fsmlist.h`:

```cpp
enum class State {
IDLE,
ZERO,
RL_walk,
SIMPLE_EXAMPLE,
};
```

They are connected to action classes in
`StateMachine/include/state_registry_user.h`:

```cpp
#define PHYBOT_FOR_EACH_STATE(M) \
    M(ZeroState, State::ZERO) \
    M(rl_deploy_cpg, State::RL_walk) \
    M(SimpleStateExample, State::SIMPLE_EXAMPLE)
```

To add another state, add a new enum value, include the action class header, and
add one more `M(ActionClass, State::YourState)` line.

## Joystick State Switch Example

The delivered closed joystick library keeps the built-in mapping small and
includes `SIMPLE_EXAMPLE` as the custom-state template:

```cpp
X -> State::ZERO
B -> State::RL_walk
A -> State::SIMPLE_EXAMPLE
```

Customers can add their own state switch in open source code by using the public
joystick APIs. `RobotStart/test/joy_test.cpp` shows the same pattern in open
source:

```cpp
void apply_custom_state_switch(Joystick& joystick) {
    xbox_map_t button_state = joystick.get_button_state();

    if (button_state.a == 1) {
        joystick.set_next_state(State::SIMPLE_EXAMPLE);
    }
}
```

Call it between `joystick.run()` and `joystick.SetDataToPackage(package)`:

```cpp
joystick.GetDataFromPackage(package);
joystick.run();
apply_custom_state_switch(joystick);
joystick.SetDataToPackage(package);
```

To connect a customer state, replace `State::SIMPLE_EXAMPLE` with the new enum
value after registering that state.

Rebuild after editing:

```bash
./autobuild.sh realrobot_mini
```

An action class used by `GenericState` should provide:

- `void GetDataFromPackage(DataPackage&)`
- `void Step()`
- `void SetDataToPackage(DataPackage&)`
- optional `void Exit()`
