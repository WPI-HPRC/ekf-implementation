#pragma once

#include "BasicLinearAlgebra.h"

#include "kfConsts.h"
#include <cstdint>
#include "QuaternionUtils.h"

/**
 * @name QMEKfInds
 * @brief Struct holding the indices of the total QMEKF
 */
namespace QMEKFInds {
constexpr uint8_t q_w = 0;
constexpr uint8_t q_x = 1;
constexpr uint8_t q_y = 2;
constexpr uint8_t q_z = 3;
constexpr std::array<uint8_t, 4> quat = {q_w, q_x, q_y, q_z};

constexpr uint8_t v_x = 4;
constexpr uint8_t v_y = 5;
constexpr uint8_t v_z = 6;
constexpr std::array<uint8_t, 4> vel = {v_x, v_y, v_z};

constexpr uint8_t p_x = 7;
constexpr uint8_t p_y = 8;
constexpr uint8_t p_z = 9;
constexpr std::array<uint8_t, 4> pos = {p_x, p_y, p_z};

constexpr uint8_t gb_x = 10;
constexpr uint8_t gb_y = 11;
constexpr uint8_t gb_z = 12;
constexpr std::array<uint8_t, 3> gyroBias = {gb_x, gb_y, gb_z};

constexpr uint8_t ab_x = 13;
constexpr uint8_t ab_y = 14;
constexpr uint8_t ab_z = 15;
constexpr std::array<uint8_t, 3> accelBias = {ab_x, ab_y, ab_z};

constexpr uint8_t mb_x = 16;
constexpr uint8_t mb_y = 17;
constexpr uint8_t mb_z = 18;
constexpr std::array<uint8_t, 3> magBias = {mb_x, mb_y, mb_z};

constexpr uint8_t bb_z = 19;
constexpr std::array<uint8_t, 1> baroBias = {bb_z};



}; // namespace QMEKFInds

#define QMEKF_LOG_DESC(X)                                                      \
    X(0, "w", p.print(state(QMEKFInds::q_w), 4))                               \
    X(1, "i", p.print(state(QMEKFInds::q_x), 4))                               \
    X(2, "j", p.print(state(QMEKFInds::q_y), 4))                               \
    X(3, "k", p.print(state(QMEKFInds::q_z), 4))                               \
	X(4, "v_x", p.print(state(QMEKFInds::v_x), 4))                             \
    X(5, "v_y", p.print(state(QMEKFInds::v_y), 4))                             \
    X(6, "v_z", p.print(state(QMEKFInds::v_z), 4))                             \
    X(7, "p_x", p.print(state(QMEKFInds::p_x), 4))                             \
	X(8, "p_y", p.print(state(QMEKFInds::p_y), 4))                             \
    X(9, "p_z", p.print(state(QMEKFInds::p_z), 4))                             \


/**
 * @name QMEKFStateEstimator
 * @author QMEKF team
 * @brief Attitude and Position/Velocity estimation. See matlab simulation for details
 */
class StateEstimator {

  public:
    // State Vector Allocation
    // Quat, vel, pos, gyroBias, accelBias, magBias, baroBias. TODO eventually change with real ECEF
    BLA::Matrix<20, 1> x;

    // Error Covariance Allocation TODO eventually
    BLA::Matrix<19, 19> P;

    /**
     * @name init
     * @author @frostydev99
     * @param x_0 - Initial State
     * @param dt  - Discrete time step
     */
    void init(BLA::Matrix<3, 1> ECEF, float curr_time);
    void padLoop(BLA::Matrix<3, 1> accel, BLA::Matrix<3, 1> mag, BLA::Matrix<3, 1> gps_pos);
    void computeInitialOrientation();

    BLA::Matrix<20, 1> fastGyroProp(BLA::Matrix<3,1> gyro, float curr_time);
    BLA::Matrix<20, 1> fastAccelProp(BLA::Matrix<3,1> accel, float curr_time);
    BLA::Matrix<19, 19> ekfPredict(float curr_time);

    // Update Functions
    BLA::Matrix<20, 1> runAccelUpdate(BLA::Matrix<3, 1> a_b, float curr_time);
    BLA::Matrix<20, 1> runMagUpdate(BLA::Matrix<3, 1> m_b, float curr_time);
	  BLA::Matrix<20, 1> runGPSUpdate(BLA::Matrix<3, 1> gpsPos, BLA::Matrix<3, 1> gpsVel);
	  BLA::Matrix<20, 1> runBaroUpdate(BLA::Matrix<1, 1> baro);

  private:
    // Identity Matrices
    BLA::Matrix<20, 20> I_20 = BLA::Eye<20, 20>();
	  BLA::Matrix<19, 19> I_19 = BLA::Eye<19, 19>();
    BLA::Matrix<3, 3> I_3 = BLA::Eye<3, 3>();

    BLA::Matrix<3, 1> launch_ecef = {1475354.0f, -4490428.0f, 4268181.0f};
    BLA::Matrix<3, 3> launch_dcmned2ecef;

    // TODO this one is more confusing because can have diff sizes matrices
    BLA::Matrix<20, 1> EKFCalcErrorInject(BLA::Matrix<20, 1> &oldState, BLA::Matrix<19, 19> &oldP, BLA::Matrix<3, 1> &sens_reading, BLA::Matrix<3, 19> H, BLA::Matrix<3, 1> h, BLA::Matrix<3, 3> R);

    BLA::Matrix<6, 1> lastTimes = {0, 0, 0, 0, 0, 0};
    // Gyro prop, accel prop, accel update, mag update, gps update, baro update
    BLA::Matrix<3,1> gyro_prev = {0, 0, 0};
    BLA::Matrix<3,1> accel_prev = {0, 0, 0};
    BLA::Matrix<3,1> mag_prev = {0, 0, 0};
    BLA::Matrix<3,1> gps_pos_prev = {0, 0, 0};
    BLA::Matrix<3,1> gps_vel_prev = {0, 0, 0};
    BLA::Matrix<1,1> baro_prev = {0};


    // padLoop vars:
    float numLoop;
    BLA::Matrix<3, 1> sumAccel;
    BLA::Matrix<3, 1> sumMag;

    // //R matricies
    // BLA::Matrix<3, 3> R_accel = {accel_var, 0, 0,
    //                             0, accel_var, 0,
    //                             0, 0, accel_var};

    // BLA::Matrix<3, 3> R_mag = {mag_var, 0, 0,
    //                           0, mag_var, 0,
    //                           0, 0, mag_var};

    // BLA::Matrix<3, 3> R_gps= {gps_var, 0, 0,
    //                          0, gps_var, 0,
    //                          0, 0, gps_var};

    // BLA::Matrix<10, 1> R_all = {
    //   pow(sqrt(asm330_const::accelXY_var) * 9.8, 2),
    //   pow(sqrt(asm330_const::accelXY_var) * 9.8, 2),
    //   pow(sqrt(asm330_const::accelZ_var) * 9.8, 2),
    //   icm20948_const::magXYZ_var,
    //   icm20948_const::magXYZ_var,
    //   icm20948_const::magXYZ_var,
    //   Max10S_const::gpsXYZ_var,
    //   Max10S_const::gpsXYZ_var,
    //   Max10S_const::gpsXYZ_var,
    //   Max10S_const::baro_var
    // };



};
