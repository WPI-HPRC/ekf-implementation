#include "kfConsts.h"

namespace earth_consts {
    BLA::Matrix<3, 1> earth_rot_ecef = {0, 0, 0.00007292}; // rad/s
}

namespace asm330_const {
    BLA::Matrix<3, 1> accel_var = {square(0.0020f), square(0.0014f), square(0.0014f)};
    BLA::Matrix<3, 1> gyro_var = {square(0.0947f), square(0.0947f), square(0.0947f)};
    BLA::Matrix<1, 1> avg_accel_var = {sum(accel_var) / 3.0f};
    BLA::Matrix<1, 1> avg_gyro_var = {sum(gyro_var) / 3.0f};
    BLA::Matrix<3, 1> loc = {-0.1, 0.05, 0.05};
}

namespace lps22_const {
    BLA::Matrix<1, 1> baro_var = {0.0f};
    BLA::Matrix<1, 1> baro_bias = {0.0f};
}

namespace icm20948_const {
    BLA::Matrix<3, 1> accel_var = {square(0.0020f), square(0.0014f), square(0.0014f)};
    BLA::Matrix<3, 1> gyro_var = {square(0.0947f), square(0.0947f), square(0.0947f)};
    BLA::Matrix<3, 1> mag_var = {1.0f, 2.0f, 3.0f};
    BLA::Matrix<3, 1> mag_bias = {0, 0, 0};
    BLA::Matrix<1, 1> avg_accel_var = {sum(accel_var) / 3.0f};
    BLA::Matrix<1, 1> avg_gyro_var = {sum(gyro_var) / 3.0f};
    BLA::Matrix<3, 1> loc = {-0.1, 0.05, 0.05};
}

namespace Max10S_const {
    BLA::Matrix<3, 1> gpsPos_var = {square(2.0f), square(2.0f), square(2.0f)};
    BLA::Matrix<3, 1> gpsVel_var = {square(0.5f), square(0.5f), square(0.5f)};
}

namespace mars_const {
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
    BLA::Matrix<2, 1> accel_avg_vars_inv_tmp = {1.0f / icm20948_const::accel_var(0), 1.0f / asm330_const::accel_var(0)};
    BLA::Matrix<2, 2> accel_avg_vars_inv = toDiag(accel_avg_vars_inv_tmp);
    BLA::Matrix<3, 2> R = hstack(icm20948_const::loc, asm330_const::loc);
    BLA::Matrix<2, 1> ones = {1, 1};
    BLA::Matrix<3, 1> r_bar = R * accel_avg_vars_inv * ones;
    BLA::Matrix<3, 3> M = R * accel_avg_vars_inv * ~R;
    BLA::Matrix<3, 3> M_pinv = pinv(M);
    BLA::Matrix<2, 1> w_hat = accel_avg_vars_inv * (ones - ~R * (M_pinv * r_bar));
    BLA::Matrix<2, 1> w_accel = w_hat / sum(w_hat);
    BLA::Matrix<3, 1> combined_accel_var = combine_variances(hstack(icm20948_const::accel_var, asm330_const::accel_var), w_accel);
    BLA::Matrix<3, 1> bodyToVIMUDis = w_accel(0) * icm20948_const::loc + w_accel(1) * asm330_const::loc;

    // VIMU Gyro:
    BLA::Matrix<2, 1> gyro_avg_vars_inv_tmp = {1.0f / icm20948_const::gyro_var(0), 1.0f / asm330_const::gyro_var(0)};
    BLA::Matrix<2, 2> gyro_avg_vars_inv = toDiag(gyro_avg_vars_inv_tmp);
    BLA::Matrix<2, 1> w_gyro = gyro_avg_vars_inv_tmp / sum(gyro_avg_vars_inv_tmp);
    BLA::Matrix<3, 1> combined_gyro_var = combine_variances(hstack(icm20948_const::gyro_var, asm330_const::gyro_var), w_gyro);

    // VIMU Mag:
    BLA::Matrix<2, 1> mag_avg_vars_inv_tmp = {1.0f / mars_const::icm_mag_var(0), 1.0f / mars_const::asm_mag_var(0)};
    BLA::Matrix<2, 2> mag_avg_vars_inv = toDiag(mag_avg_vars_inv_tmp);
    BLA::Matrix<2, 1> w_mag = mag_avg_vars_inv_tmp / sum(mag_avg_vars_inv_tmp);
    BLA::Matrix<3, 1> combined_mag_var = combine_variances(hstack(mars_const::icm_mag_var, mars_const::asm_mag_var), w_mag);
}

namespace vimu_const {
    BLA::Matrix<2, 1> ones = {1, 1};
    BLA::Matrix<2, 2> avg_accel_vars_inv = toDiag(vstack(icm20948_const::avg_accel_var, asm330_const::avg_accel_var));
    BLA::Matrix<3, 2> R = hstack(icm20948_const::loc, asm330_const::loc);
    BLA::Matrix<3, 1> r_bar = R * avg_accel_vars_inv * ones;
    BLA::Matrix<3, 3> M = R * avg_accel_vars_inv * ~R;
    BLA::Matrix<2, 2> avg_gyro_vars_inv = BLA::Inverse(toDiag(vstack(icm20948_const::avg_gyro_var, asm330_const::avg_gyro_var)));
    BLA::Matrix<3, 1> accel_var = {square(0.0020f), square(0.0014f), square(0.0014f)};
    BLA::Matrix<3, 1> gyro_var = {square(0.0947f), square(0.0947f), square(0.0947f)};
    BLA::Matrix<3, 1> mag_var = {1.0f, 2.0f, 3.0f};
    BLA::Matrix<3, 1> accel_bias = {0, 0, 0};
    BLA::Matrix<3, 1> gyro_bias = {0, 0, 0};
}
