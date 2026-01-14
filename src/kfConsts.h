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
    BLA::Matrix<3, 1> earth_rot_ecef = {0, 0, 0.00007292}; // rad/s

}

namespace mars_const {
    // BLA::Matrix<3, 3> R_BM = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    // BLA::Matrix<3, 1> p_MB_B = {-0.5, 0.1, }
    // No, TODO: Do as dual quaternions

    BLA::Matrix<3, 1> bodyToMarsDis = {-0.3, 0.1, 0.1};
    BLA::Matrix<4, 1> bodyToMarsOrientation = {1, 0, 0, 0};


    // ASM IMU:
    BLA::Matrix<3, 1> marsToASMDis = {0.1, 0.1, 0.1};
    BLA::Matrix<4, 1> marsToASMOrientation = {1, 0, 0, 0};
    BLA::Matrix<3, 1> bodyToASMDis = bodyToMarsDis + qRot(bodyToMarsOrientation, marsToASMDis);
    BLA::Matrix<4, 1> bodyToASMOrientation = quatMultiply(bodyToMarsOrientation, marsToASMOrientation);

    // ASM gyro
    BLA::Matrix<3, 1> asm_gyro_var = {square(0.0020f), square(0.0014f), square(0.0014f)};
    BLA::Matrix<3, 1> asm_gyro_bias_var = {0, 0, 0};

    // ASM accel
    BLA::Matrix<3, 1> asm_accel_var = {square(0.0020f), square(0.0014f), square(0.0014f)};
    BLA::Matrix<3, 1> asm_accel_bias_var = {0, 0, 0};

    // ASM mag
    BLA::Matrix<3, 1> asm_mag_var = {square(0.0020f), square(0.0014f), square(0.0014f)};



    // ICM IMU:
    BLA::Matrix<3, 1> marsToICMDis = {0.1, -0.1, -0.1};
    BLA::Matrix<4, 1> marsToICMOrientation = {1, 0, 0, 0};

    // ICM gyro
    BLA::Matrix<3, 1> icm_gyro_var = {square(0.0020f), square(0.0014f), square(0.0014f)};
    BLA::Matrix<3, 1> icm_gyro_bias_var = {0, 0, 0};

    // ICM accel
    BLA::Matrix<3, 1> icm_accel_var = {square(0.0020f), square(0.0014f), square(0.0014f)};
    BLA::Matrix<3, 1> icm_accel_bias_var = {0, 0, 0};

    // ICM mag
    BLA::Matrix<3, 1> icm_mag_var = {square(0.0020f), square(0.0014f), square(0.0014f)};




    // LPS Baro:

    BLA::Matrix<3, 1> marsToLPSDis = {0, 0, 0};
    BLA::Matrix<1, 1> LPS_var = {square(2.0f)};


    // GPS:
    BLA::Matrix<3, 1> marsToGPSDis = {0.1, -0.1, -0.1};
    BLA::Matrix<3, 1> gps_pos_var_ned = {square(0.0020f), square(0.0014f), square(0.0014f)};
    BLA::Matrix<3, 1> gps_vel_var_ned = {0, 0, 0};


    // VIMU:
    // TODO one day model bias instability

    // VIMU Accel:
    BLA::Matrix<2, 1> accel_avg_vars_inv_tmp = {1.0f / icm_accel_var(0), 1.0f / asm_accel_var(0)};
    BLA::Matrix<2, 2> accel_avg_vars_inv = toDiag(accel_avg_vars_inv_tmp);
    BLA::Matrix<3, 2> R = hstack(bodyToASMDis, bodyToASMDis);

    BLA::Matrix<2, 1> ones = {1, 1};
    BLA::Matrix<3, 1> r_bar = R * accel_avg_vars_inv * ones;
    BLA::Matrix<3, 3> M = R * accel_avg_vars_inv * ~R;
    BLA::Matrix<3, 3> M_pinv = pinv(M);
    BLA::Matrix<2, 1> w_hat = accel_avg_vars_inv * (ones - ~R * (M_pinv * r_bar));
    BLA::Matrix<2, 1> w_accel = w_hat / sum(w_hat);

    BLA::Matrix<3, 1> combined_accel_var = combine_variances(hstack(icm_accel_var, asm_accel_var), w_accel);
    BLA::Matrix<3, 1> bodyToVIMUDis = w_accel(0) * bodyToASMDis + w_accel(1) * bodyToASMDis;

    // VIMU Gyro:

    BLA::Matrix<2, 1> gyro_avg_vars_inv_tmp = {1.0f / icm_gyro_var(0), 1.0f / asm_gyro_var(0)};
    BLA::Matrix<2, 2> gyro_avg_vars_inv = toDiag(gyro_avg_vars_inv_tmp);
    BLA::Matrix<2, 1> w_gyro = gyro_avg_vars_inv_tmp / sum(gyro_avg_vars_inv_tmp);

    BLA::Matrix<3, 1> combined_gyro_var = combine_variances(hstack(icm_gyro_var, asm_gyro_var), w_gyro);


    // VIMU Mag (Same as gyro):

    BLA::Matrix<2, 1> mag_avg_vars_inv_tmp = {1.0f / icm_mag_var(0), 1.0f / asm_mag_var(0)};
    BLA::Matrix<2, 2> mag_avg_vars_inv = toDiag(mag_avg_vars_inv_tmp);
    BLA::Matrix<2, 1> w_mag = mag_avg_vars_inv_tmp / sum(mag_avg_vars_inv_tmp);

    BLA::Matrix<3, 1> combined_mag_var = combine_variances(hstack(icm_mag_var, asm_mag_var), w_mag);


    // VBaro:

    // TODO one day when we have a lot of barometers. Same thing tho
    // BLA::Matrix<1, 2> vbaro_weights = {0, 0};
    // BLA::Matrix<3, 1> marsToVBaroDis = {0, 0, 0};


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
