// #pragma once

// #include "BasicLinearAlgebra.h"

// #include "kfConsts.h"
// #include <cstdint>
// #include "QuaternionUtils.h"

// /**
//  * @name QMEKfInds
//  * @brief Struct holding the indices of the total QMEKF
//  */
// namespace QMEKFInds {
// constexpr uint8_t q_w = 0;
// constexpr uint8_t q_x = 1;
// constexpr uint8_t q_y = 2;
// constexpr uint8_t q_z = 3;
// constexpr std::array<uint8_t, 4> quat = {q_w, q_x, q_y, q_z};

// constexpr uint8_t v_x = 4;
// constexpr uint8_t v_y = 5;
// constexpr uint8_t v_z = 6;
// constexpr std::array<uint8_t, 4> vel = {v_x, v_y, v_z};

// constexpr uint8_t p_x = 7;
// constexpr uint8_t p_y = 8;
// constexpr uint8_t p_z = 9;
// constexpr std::array<uint8_t, 4> pos = {p_x, p_y, p_z};

// constexpr uint8_t gb_x = 10;
// constexpr uint8_t gb_y = 11;
// constexpr uint8_t gb_z = 12;
// constexpr std::array<uint8_t, 3> gyroBias = {gb_x, gb_y, gb_z};

// constexpr uint8_t ab_x = 13;
// constexpr uint8_t ab_y = 14;
// constexpr uint8_t ab_z = 15;
// constexpr std::array<uint8_t, 3> accelBias = {ab_x, ab_y, ab_z};

// constexpr uint8_t mb_x = 16;
// constexpr uint8_t mb_y = 17;
// constexpr uint8_t mb_z = 18;
// constexpr std::array<uint8_t, 3> magBias = {mb_x, mb_y, mb_z};

// constexpr uint8_t bb_z = 19;
// constexpr std::array<uint8_t, 1> baroBias = {bb_z};



// }; // namespace QMEKFInds

// #define QMEKF_LOG_DESC(X)                                                      \
//     X(0, "w", p.print(state(QMEKFInds::q_w), 4))                               \
//     X(1, "i", p.print(state(QMEKFInds::q_x), 4))                               \
//     X(2, "j", p.print(state(QMEKFInds::q_y), 4))                               \
//     X(3, "k", p.print(state(QMEKFInds::q_z), 4))                               \
// 	X(4, "v_x", p.print(state(QMEKFInds::v_x), 4))                             \
//     X(5, "v_y", p.print(state(QMEKFInds::v_y), 4))                             \
//     X(6, "v_z", p.print(state(QMEKFInds::v_z), 4))                             \
//     X(7, "p_x", p.print(state(QMEKFInds::p_x), 4))                             \
// 	X(8, "p_y", p.print(state(QMEKFInds::p_y), 4))                             \
//     X(9, "p_z", p.print(state(QMEKFInds::p_z), 4))                             \


// /**
//  * @name QMEKFStateEstimator
//  * @author QMEKF team
//  * @brief Attitude and Position/Velocity estimation. See matlab simulation for details
//  */
// class StateEstimator {

//   public:
//     StateEstimator(const TimedPointer<ICMData>, const TimedPointer<MAX10SData>, float dt);

//     /**
//      * @name init
//      * @author @frostydev99
//      * @param x_0 - Initial State
//      * @param dt  - Discrete time step
//      */
//     void init(BLA::Matrix<3, 1> LLA);

//     /**
//      * @name onLoop
//      * @author @frostydev99
//      * @brief Run Every Loop
//      * @paragraph This method should run every loop of the expected prediction
//      * update rate given by dt
//      */
//     BLA::Matrix<20, 1> onLoop(int state);

//   private:
//     const TimedPointer<ICMData> IMUData;
//     const TimedPointer<MAX10SData> gpsData;

//     // Prediction Functions
//     // BLA::Matrix<20, 1> predictionFunction(BLA::Matrix<20, 1> x,
//     //                                       BLA::Matrix<3, 1> gyro,
// 		// 								  BLA::Matrix<3, 1> accel);
//     // BLA::Matrix<19, 19> predictionJacobian(BLA::Matrix<20, 1> x,
// 		// 								  BLA::Matrix<3, 1> gyro,
// 		// 								  BLA::Matrix<3, 1> accel);

//     BLA::Matrix<20,1> fastIMUProp(BLA::Matrix<3,1> gyro, BLA::Matrix<3, 1> accel, float att_dt, float pv_dt);
//     BLA::Matrix<19, 19> predictionFunction(BLA::Matrix<19, 19> P_, BLA::Matrix<3, 1> accelVec, BLA::Matrix<3, 1> gyroVec, float dt);

//     // Update Functions
//     BLA::Matrix<20,1> runAccelUpdate(BLA::Matrix<20, 1> &x, BLA::Matrix<3, 1> a_b);

//     BLA::Matrix<20,1> runMagUpdate(BLA::Matrix<20, 1> &x, BLA::Matrix<3, 1> m_b);
	
// 	  BLA::Matrix<20,1> runGPSUpdate(BLA::Matrix<20, 1> &x, BLA::Matrix<3, 1> gps);
	
// 	  // void runBaroUpdate(BLA::Matrix<20, 1> &x, BLA::Matrix<1, 1> baro);

//     BLA::Matrix<20,1> EKFCalcErrorInject(BLA::Matrix<20, 1> &oldState, BLA::Matrix<19, 19> &oldP, BLA::Matrix<3, 1> &sens_reading, BLA::Matrix<3, 19> H, BLA::Matrix<3, 1> h, BLA::Matrix<3, 3> R);

//     // State Vector Allocation
//     BLA::Matrix<20, 1> x;

//     // Error Covariance Allocation
//     BLA::Matrix<19, 19> P;

//     // Identity Matrices
//     BLA::Matrix<20, 20> I_20 = BLA::Eye<20, 20>();
// 	BLA::Matrix<19, 19> I_19 = BLA::Eye<19, 19>();
//     BLA::Matrix<3, 3> I_3 = BLA::Eye<3, 3>();

//     //R values
//     float accel_var = pow(sqrt(asm330_const::accelXY_var) * 9.8, 2);
//     float mag_var = icm20948_const::magXYZ_var;
//     float gps_var = Max10S_const::gpsXYZ_var;
//     //R matricies
//     BLA::Matrix<3, 3> R_accel = {accel_var, 0, 0,
//                                 0, accel_var, 0,
//                                 0, 0, accel_var};

//     BLA::Matrix<3, 3> R_mag = {mag_var, 0, 0,
//                               0, mag_var, 0,
//                               0, 0, mag_var};

//     BLA::Matrix<3, 3> R_gps= {gps_var, 0, 0,
//                              0, gps_var, 0,
//                              0, 0, gps_var};

//     BLA::Matrix<10, 1> R_all = {
//       pow(sqrt(asm330_const::accelXY_var) * 9.8, 2),
//       pow(sqrt(asm330_const::accelXY_var) * 9.8, 2),
//       pow(sqrt(asm330_const::accelZ_var) * 9.8, 2),
//       icm20948_const::magXYZ_var,
//       icm20948_const::magXYZ_var,
//       icm20948_const::magXYZ_var,
//       Max10S_const::gpsXYZ_var,
//       Max10S_const::gpsXYZ_var,
//       Max10S_const::gpsXYZ_var,
//       Max10S_const::baro_var
//     };


// 	BLA::Matrix<5, 1> lastTimes = {0, 0, 0, 0, 0, 0, 0};
//     // Gyro prop, accel prop, accel update, mag update, gps update, baro update

//     BLA::Matrix<3,1> gyro_prev = {0, 0, 0};
//     BLA::Matrix<3,1> accel_prev = {0, 0, 0};
//     BLA::Matrix<3,1> mag_prev = {0, 0, 0};
//     BLA::Matrix<3,1> gps_pos_prev = {0, 0, 0};
//     BLA::Matrix<3,1> gps_vel_prev = {0, 0, 0};
//     BLA::Matrix<1,1> baro_prev = {0};

//   BLA::Matrix<3, 1> normal_i = {0, 0, -9.8037}; // [m/s^2]
//   BLA::Matrix<3, 1> g_i = {0, 0, 9.8037}; // [m/s^2]
//   BLA::Matrix<3, 1> m_i = {18.659605, -4.540227, 49.09786}; // [uT] Kids rocket params
//   BLA::Matrix<3, 1> launch_ecef = {1311800, -4337300, 4473600}; // [m] // asuming 0 above surface
//   BLA::Matrix<3, 1> launch_lla = {44.825070, -73.171726, 0}; // [whatever tf units are in (deg, deg, m)]
//   BLA::Matrix<2, 1> launch_ll = {launch_lla(0), launch_lla(1)};
//   BLA::Matrix<3, 3> R_ET = QuaternionUtils::dcm_ned2ecef(launch_ll);

// template <size_t N, size_t M>
// BLA::Matrix<M, 1> extractSub(const BLA::Matrix<N, 1> &x,
//                              const std::array<uint8_t, M> &inds) {
//     BLA::Matrix<M, 1> sub;
//     for (int i = 0; i < M; i++) {
//         sub(i) = x(inds[i]);
//     }
//     return sub;
// }

// BLA::Matrix<3, 3> quat2rot(const BLA::Matrix<4, 1> &q);

// };
