#pragma once

constexpr static float g = 9.80665; // [m/s/s] Earth's Grav Accel

constexpr static struct {
    float accelXY_var = 0.0020f; // [g]
    float accelZ_var  = 0.0014f; // [g]
    float accelXY_VRW = 0.0003f; // [g/sqrt(Hz)]
    float accelZ_VRW  = 0.0002f; // [g/sqrt(Hz)]
    float gyroX_var   = 0.0947f; // [deg/s]
    float gyroY_var   = 0.1474f; // [deg/s]
    float gyroZ_var   = 0.2144f; // [deg/s]
    float gyro_VRW    = 0.0241f; // [deg/s/sqrt(Hz)]
} asm330_const;

constexpr static struct {
    float baro_var = 0.0f;
} lps22_const;

constexpr static struct {
    float accelXY_var = 0.0383f; // [g]
    float accelZ_var  = 0.0626f; // [g]
    float accelXY_VRW = 0.0062f; // [g/sqrt(Hz)]
    float accelZ_VRW  = 0.0099f; // [g/sqrt(Hz)]
    float gyroXYZ_var = 0.0051f; // [deg/s]
    float gyro_VRW    = 8.33e-4f; // [deg/s/sqrt(Hz)]
    float magXYZ_var  = 0.7263f; // [uT]
} icm20948_const;