#pragma once

constexpr static struct {
    float accelXYZ_var = 0.0f;
    float gyroXYZ_var  = 0.0f;
} asm330_const;

constexpr static struct {
    float baro_var = 0.0f;
} lps22_const;

constexpr static struct {
    float accelXYZ_var = 0.0f;
    float gyroXYZ_var  = 0.0f;
    float magXYZ_var   = 0.0f;
} icm20948_const;