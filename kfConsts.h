#pragma once

#include "Arduino.h"

constexpr static float g = 9.80665; // [m/s/s] Earth's Grav Accel

constexpr static struct {
    float accelXY_var = powf(0.0020f, 2); // [g]
    float accelZ_var  = powf(0.0014f, 2.0f); // [g]
    float accelXY_VRW = 0.0003f; // [g/sqrt(Hz)]
    float accelZ_VRW  = 0.0002f; // [g/sqrt(Hz)]
    float gyroX_var   = powf(0.0947f, 2); // [deg/s]
    float gyroY_var   = powf(0.1474f, 2); // [deg/s]
    float gyroZ_var   = powf(0.2144f, 2); // [deg/s]
    float gyro_VRW    = 0.0241f; // [deg/s/sqrt(Hz)]
} asm330_const;

constexpr static struct {
    float baro_var = 0.0f;
} lps22_const;

constexpr static struct {
    float accelXY_var = powf(0.0383f, 2); // [g]
    float accelZ_var  = powf(0.0626f, 2); // [g]
    float accelXY_VRW = 0.0062f; // [g/sqrt(Hz)]
    float accelZ_VRW  = 0.0099f; // [g/sqrt(Hz)]
    // float gyroXYZ_var = powf(0.0051, 2); // [rad/s]
    float gyroXYZ_var = powf(5e-4, 2); // [rad/s]
    float gyro_VRW    = 8.33e-4f; // [rad/s/sqrt(Hz)]
    float magXYZ_var  = powf(0.7263f, 2); // [uT]
    float quatVar     = 0.3; // Idk Guess
} icm20948_const;