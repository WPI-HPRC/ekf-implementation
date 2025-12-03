#pragma once

#include "BasicLinearAlgebra.h"
#include <cmath>
constexpr static float g = 9.80665; // [m/s/s] Earth's Grav Accel

constexpr float square(float a) { return a * a; }

namespace asm330_const {
constexpr float accelXY_var = square(0.0020f); // [g]
constexpr float accelZ_var = square(0.0014f);  // [g]
constexpr float accelXY_VRW = 0.0003f;         // [g/sqrt(Hz)]
constexpr float accelZ_VRW = 0.0002f;          // [g/sqrt(Hz)]
constexpr float gyroX_var = square(0.0947f);   // [deg/s]
constexpr float gyroY_var = square(0.1474f);   // [deg/s]
constexpr float gyroZ_var = square(0.2144f);   // [deg/s]
constexpr float gyro_VRW = 0.0241f;            // [deg/s/sqrt(Hz)]
}; // namespace asm330_const

namespace lps22_const {
constexpr float baro_var = 0.0f;
};

namespace icm20948_const {
constexpr float accelXY_var = square(0.0383f); // [g]
constexpr float accelZ_var = square(0.0626f);  // [g]
constexpr float accelXY_VRW = 0.0062f;         // [g/sqrt(Hz)]
constexpr float accelZ_VRW = 0.0099f;          // [g/sqrt(Hz)]
// constexpr float gyroXYZ_var = square(0.0051); // [rad/s]
constexpr float gyroXYZ_var = square(5e-4);   // [rad/s]
constexpr float gyro_VRW = 8.33e-4f;          // [rad/s/sqrt(Hz)]
constexpr float magXYZ_var = square(0.7263f); // [uT]
constexpr float quatVar = 0.3;                // Idk Guess


}; // namespace icm20948_const


namespace Max10S_const {
    constexpr float gpsXYZ_var = square(2.0); // [m] idk guess
    constexpr float baro_var = square(7.5); // [P] idk guess
}

namespace vimu_const {
    // TODO vimu/2imu

    constexpr float gyro_var = 0.0051;
    constexpr float gyro_bias_var = square(4.9e-5f * 9.8f);

    BLA::Matrix<3, 1> accel_var = {square(4.9e-5f * 9.9f), square(4.9e-5f * 9.9f), square(4.9e-5f * 9.9f)};
    constexpr float accel_bias_var = square(0.00098f * 9.8f);

    constexpr float mag_bias_var = 25;
    constexpr float baro_bias_var = square(7.5f);


}
