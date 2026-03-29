#pragma once

#include "BasicLinearAlgebra.h"
#include <cmath>
#include "QuaternionUtils.h"

using namespace QuaternionUtils;

constexpr float square(float a) { return a * a; }

namespace earth_consts {
    // Baro:
    constexpr float P0 = 101325.0; // [P] trad sea level (word on the street)
    constexpr float L = -0.0065;
    constexpr float T0 = 288.15;
    constexpr float M = 0.0289644;
    constexpr float R = 8.3144598;

    // Grav: (until world model implemented)
    constexpr float g_e = 9.80665; // [m/s^2]

    // TODO will want better geodesy if going high up or using inertial mindset
    extern BLA::Matrix<3, 1> earth_rot_ecef; // rad/s
}

namespace mars_const {
    extern BLA::Matrix<3, 1> bodyToMarsDis;
    extern BLA::Matrix<4, 1> bodyToMarsOrientation;

    // ASM IMU:
    extern BLA::Matrix<3, 1> marsToASMDis;
    extern BLA::Matrix<4, 1> marsToASMOrientation;
    extern BLA::Matrix<3, 1> bodyToASMDis;
    extern BLA::Matrix<4, 1> bodyToASMOrientation;

    // ASM gyro
    extern BLA::Matrix<3, 1> asm_gyro_var;
    extern BLA::Matrix<3, 1> asm_gyro_bias_var;

    // ASM accel
    extern BLA::Matrix<3, 1> asm_accel_var;
    extern BLA::Matrix<3, 1> asm_accel_bias_var;

    // ASM mag
    extern BLA::Matrix<3, 1> asm_mag_var;

    // ICM IMU:
    extern BLA::Matrix<3, 1> marsToICMDis;
    extern BLA::Matrix<4, 1> marsToICMOrientation;

    // ICM gyro
    extern BLA::Matrix<3, 1> icm_gyro_var;
    extern BLA::Matrix<3, 1> icm_gyro_bias_var;

    // ICM accel
    extern BLA::Matrix<3, 1> icm_accel_var;
    extern BLA::Matrix<3, 1> icm_accel_bias_var;

    // ICM mag
    extern BLA::Matrix<3, 1> icm_mag_var;

    // LPS Baro:
    extern BLA::Matrix<3, 1> marsToLPSDis;
    extern BLA::Matrix<1, 1> LPS_var;

    // GPS:
    extern BLA::Matrix<3, 1> marsToGPSDis;
    extern BLA::Matrix<3, 1> gps_pos_var_ned;
    extern BLA::Matrix<3, 1> gps_vel_var_ned;

    // VIMU:
    extern BLA::Matrix<2, 1> accel_avg_vars_inv_tmp;
    extern BLA::Matrix<2, 2> accel_avg_vars_inv;
    extern BLA::Matrix<3, 2> R;
    extern BLA::Matrix<2, 1> ones;
    extern BLA::Matrix<3, 1> r_bar;
    extern BLA::Matrix<3, 3> M;
    extern BLA::Matrix<3, 3> M_pinv;
    extern BLA::Matrix<2, 1> w_hat;
    extern BLA::Matrix<2, 1> w_accel;
    extern BLA::Matrix<3, 1> combined_accel_var;
    extern BLA::Matrix<3, 1> bodyToVIMUDis;

    // VIMU Gyro:
    extern BLA::Matrix<2, 1> gyro_avg_vars_inv_tmp;
    extern BLA::Matrix<2, 2> gyro_avg_vars_inv;
    extern BLA::Matrix<2, 1> w_gyro;
    extern BLA::Matrix<3, 1> combined_gyro_var;

    // VIMU Mag:
    extern BLA::Matrix<2, 1> mag_avg_vars_inv_tmp;
    extern BLA::Matrix<2, 2> mag_avg_vars_inv;
    extern BLA::Matrix<2, 1> w_mag;
    extern BLA::Matrix<3, 1> combined_mag_var;
}

namespace asm330_const {
    constexpr float accelXY_var = square(0.0020f); // [g]
    constexpr float accelZ_var = square(0.0014f);  // [g]
    constexpr float accelXY_VRW = 0.0003f;         // [g/sqrt(Hz)]
    constexpr float accelZ_VRW = 0.0002f;          // [g/sqrt(Hz)]
    constexpr float gyroX_var = square(0.0947f);   // [deg/s]
    constexpr float gyroY_var = square(0.1474f);   // [deg/s]
    constexpr float gyroZ_var = square(0.2144f);   // [deg/s]
    constexpr float gyro_VRW = 0.0241f;            // [deg/s/sqrt(Hz)]

    extern BLA::Matrix<3, 1> accel_var;
    extern BLA::Matrix<3, 1> gyro_var;
    extern BLA::Matrix<1, 1> avg_accel_var;
    extern BLA::Matrix<1, 1> avg_gyro_var;
    extern BLA::Matrix<3, 1> loc;
}

namespace lps22_const {
    extern BLA::Matrix<1, 1> baro_var;
    extern BLA::Matrix<1, 1> baro_bias;

    constexpr float P0 = 101325.0; // [P] trad sea level
    constexpr float L = -0.0065;
    constexpr float T0 = 288.15;
    constexpr float M = 0.0289644;
    constexpr float R = 8.3144598;
    constexpr float g_e = 9.80665; // [m/s^2]
}

namespace icm20948_const {
    constexpr float accelXY_var = square(0.0383f); // [g]
    constexpr float accelZ_var = square(0.0626f);  // [g]
    constexpr float accelXY_VRW = 0.0062f;         // [g/sqrt(Hz)]
    constexpr float accelZ_VRW = 0.0099f;          // [g/sqrt(Hz)]
    constexpr float gyroXYZ_var = square(5e-4);    // [rad/s]
    constexpr float gyro_VRW = 8.33e-4f;           // [rad/s/sqrt(Hz)]
    constexpr float magXYZ_var = square(0.7263f);  // [uT]
    constexpr float quatVar = 0.3;                 // Idk Guess

    extern BLA::Matrix<3, 1> accel_var;
    extern BLA::Matrix<3, 1> gyro_var;
    extern BLA::Matrix<3, 1> mag_var;
    extern BLA::Matrix<3, 1> mag_bias;
    extern BLA::Matrix<1, 1> avg_accel_var;
    extern BLA::Matrix<1, 1> avg_gyro_var;
    extern BLA::Matrix<3, 1> loc;
}

namespace Max10S_const {
    extern BLA::Matrix<3, 1> gpsPos_var;
    extern BLA::Matrix<3, 1> gpsVel_var;
}

namespace vimu_const {
    extern BLA::Matrix<2, 1> ones;
    extern BLA::Matrix<2, 2> avg_accel_vars_inv;
    extern BLA::Matrix<3, 2> R;
    extern BLA::Matrix<3, 1> r_bar;
    extern BLA::Matrix<3, 3> M;
    extern BLA::Matrix<2, 2> avg_gyro_vars_inv;
    extern BLA::Matrix<3, 1> accel_var;
    extern BLA::Matrix<3, 1> gyro_var;
    extern BLA::Matrix<3, 1> mag_var;
    extern BLA::Matrix<3, 1> accel_bias;
    extern BLA::Matrix<3, 1> gyro_bias;
}