#pragma once

#include <memory>
#include <string>

#include "Joystick/include/joystick_source.h"

/// Create the active joystick backend from Joystick/config/joystick_source.yaml (joystick_backend: airplane|xbox).
std::unique_ptr<IJoystickSource> create_joystick_source(
    const std::string& config_path = "../Joystick/config/joystick_source.yaml");
