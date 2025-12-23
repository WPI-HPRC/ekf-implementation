#pragma once

#include "BasicLinearAlgebra.h"
#include <cmath>
#include "QuaternionUtils.h"

using namespace QuaternionUtils;

constexpr static float g = 9.80665; // [m/s/s] Earth's Grav Accel TODO eventually take out when have wgs model
// TODO: Fill out this file with correct terms and our units
constexpr float square(float a) { return a * a; }

namespace mars_const {
    // BLA::Matrix<3, 3> R_BM = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    // BLA::Matrix<3, 1> p_MB_B = {-0.5, 0.1, }
    // No, TODO: Do as dual quaternions
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


BLA::Matrix<3, 1> accel_var = {square(0.0020f), square(0.0014f), square(0.0014f)}; // TODO these are in g's, mode to m/s^2
BLA::Matrix<3, 1> gyro_var = {square(0.0947f), square(0.0947f), square(0.0947f)}; // TODO these are in deg, mode to rads

BLA::Matrix<1, 1> avg_accel_var = {sum(accel_var) / 3.0f};
BLA::Matrix<1, 1> avg_gyro_var = {sum(gyro_var) / 3.0f};

BLA::Matrix<3, 1> loc = {-0.1, 0.05, 0.05};

}; // namespace asm330_const

namespace lps22_const {
BLA::Matrix<1, 1> baro_var = {0.0f}; // uh what, why is it 0
BLA::Matrix<1, 1> baro_bias = {0.0f};

constexpr float P0 = 101325.0; // [P] trad sea level
constexpr float L = -0.0065;
constexpr float T0 = 288.15;
constexpr float M = 0.0289644;
constexpr float R = 8.3144598;
constexpr float g_e = 9.80665; // [m/s^2]

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

BLA::Matrix<3, 1> accel_var = {square(0.0020f), square(0.0014f), square(0.0014f)}; // TODO these are in g's, mode to m/s^2
BLA::Matrix<3, 1> gyro_var = {square(0.0947f), square(0.0947f), square(0.0947f)}; // TODO these are in deg, mode to rads
BLA::Matrix<3, 1> mag_var = {1.0f, 2.0f, 3.0f}; // [uT]

BLA::Matrix<3, 1> mag_bias =  {0, 0, 0};

BLA::Matrix<1, 1> avg_accel_var = {sum(accel_var) / 3.0f};
BLA::Matrix<1, 1> avg_gyro_var = {sum(gyro_var) / 3.0f};

BLA::Matrix<3, 1> loc = {-0.1, 0.05, 0.05};
}; // namespace icm20948_const


namespace Max10S_const {
    BLA::Matrix<3, 1> gpsPos_var = {square(2.0f), square(2.0f), square(2.0f)}; // [m] idk guess
    BLA::Matrix<3, 1> gpsVel_var = {square(0.5f), square(0.5f), square(0.5f)}; // [m/s]
}

namespace vimu_const {
    // TODO vimu/2imu
    BLA::Matrix<2, 1> ones = {1, 1};

    // TODO implement algorithm here (or don't and just do it in matlab script). For right now, just using ICM values
    
    // TODO for this implement getting the inverse (1 / ) for each entry
    BLA::Matrix<2, 2> avg_accel_vars_inv = toDiag(vstack(icm20948_const::avg_accel_var, asm330_const::avg_accel_var));

    BLA::Matrix<3, 2> R = hstack(icm20948_const::loc, asm330_const::loc);

    BLA::Matrix<3, 1> r_bar = R * avg_accel_vars_inv * ones;

    BLA::Matrix<3, 3> M = R * avg_accel_vars_inv * ~R;

    // BLA::Matrix<2, 1> w_hat = avg_accel_vars_inv * (ones - ~R * (M_pinv * r_bar));

    // BLA::Matrix<2, 1> w_accel = w_hat / sum(w_hat);

    // TODO combine accel vars


    BLA::Matrix<2, 2> avg_gyro_vars_inv = BLA::Inverse(toDiag(vstack(icm20948_const::avg_gyro_var, asm330_const::avg_gyro_var)));
    // TODO do it correctly

    // BLA::Matrix<2, 1> w_gyro = avg_gyro_vars_inv / 





    BLA::Matrix<3, 1> accel_var = {square(0.0020f), square(0.0014f), square(0.0014f)}; // TODO these are in g's, mode to m/s^2
    BLA::Matrix<3, 1> gyro_var = {square(0.0947f), square(0.0947f), square(0.0947f)}; // TODO these are in deg, mode to rads
    BLA::Matrix<3, 1> mag_var = {1.0f, 2.0f, 3.0f};

    BLA::Matrix<3, 1> accel_bias = {0, 0, 0};
    BLA::Matrix<3, 1> gyro_bias = {0, 0, 0};

}
