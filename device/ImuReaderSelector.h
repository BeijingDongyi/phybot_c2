#pragma once

#if defined(DEVICE_IMU_HIPNUC)

#include "device/Imu_hipnuc/linux/HipnucReader.h"
using ActiveImuReader = HipnucReader;

#elif defined(DEVICE_IMU_CUSTOM)

#include "device/IMU/IMUReader.h"
using ActiveImuReader = IMUReader;

#endif