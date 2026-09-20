#pragma once

#include "DataPackage/include/DataPackage.h"

/// Common interface for external joystick backends (airplane SBUS, Xbox /dev/input/js*, ...).
class IJoystickSource {
public:
    virtual ~IJoystickSource() = default;

    virtual void Init() = 0;
    virtual void Run() = 0;
    virtual void GetDataFromPackage(DataPackage& datapackage) = 0;
    virtual void SetDataToPackage(DataPackage& datapackage) = 0;
    virtual const char* backend_name() const = 0;
};
