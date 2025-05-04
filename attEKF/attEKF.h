#pragma once

#include "Arduino.h"
#include "BasicLinearAlgebra.h"

#include "../kfConsts.h"

/**
 * @name AttKfInds
 * @brief Struct holding the indices of the Attitude EKF
 */
struct AttKFInds {
    constexpr static uint8_t q_w = 0;
    constexpr static uint8_t q_x = 1;
    constexpr static uint8_t q_y = 2;
    constexpr static uint8_t q_z = 3;
    constexpr static std::array<uint8_t, 4> quat      = {q_w, q_x, q_y, q_z};

    constexpr static uint8_t gb_x = 4;
    constexpr static uint8_t gb_y = 5;
    constexpr static uint8_t gb_z = 6;
    constexpr static std::array<uint8_t, 3> gyroBias  = {gb_x, gb_y, gb_z};

    constexpr static uint8_t ab_x = 7;
    constexpr static uint8_t ab_y = 8;
    constexpr static uint8_t ab_z = 9;
    constexpr static std::array<uint8_t, 3> accelBias = {ab_x, ab_y, ab_z};

    constexpr static uint8_t mb_x = 10;
    constexpr static uint8_t mb_y = 11;
    constexpr static uint8_t mb_z = 12;
    constexpr static std::array<uint8_t, 3> magBias = {mb_x, mb_y, mb_z};
};

/**
 * @name AttStateEstimator
 * @author @frostydev99
 * @brief Quaternion Attitude State Estimator
 */
class AttStateEstimator {

    public:

    AttStateEstimator() = default;

    /**
     * @name init
     * @author @frostydev99
     * @param x_0 - Initial Quaternion State
     * @param dt  - Discrete time step
     */
    void init(BLA::Matrix<13,1> x_0, float dt);

    /**
     * @name onLoop
     * @author @frostydev99
     * @brief Run Every Loop
     * @paragraph This method should run every loop of the expected prediction update rate given by dt
     */
    void onLoop(Utility::TelemPacket sensorPacket);

    private:

    // Prediction Functions
    BLA::Matrix<13,1> predictionFunction(BLA::Matrix<13,1> x, BLA::Matrix<3,1> u);
    BLA::Matrix<13,13> predictionJacobian(BLA::Matrix<3,1> u);

    // Update Functions
    void applyGravUpdate(BLA::Matrix<3,1> a_b);

    void applyMagUpdate(BLA::Matrix<3,1> m_b);

    /**
     * @name propRK4
     * @author @frostydev99
     * @brief Runs a RKF propagation algorithm to predict state
     * @param u - [3x1] Vector of gyro readings
     */
    BLA::Matrix<13,1> propRK4(BLA::Matrix<3,1> u);

    float dt = 0.0f;

    // State Vector Allocation
    BLA::Matrix<13,1> x_min;

    BLA::Matrix<13,1> x;

    // Error Covariance Allocation
    BLA::Matrix<13,13> P;

    BLA::Matrix<13,13> P_min;

    // Process Noise Covariance Allocation
    BLA::Matrix<13,13> Q;

    // Previous control input
    BLA::Matrix<3,1> u_prev = {0,0,0,0};

    bool hasPassedGo = 0;
};

template <size_t N, size_t M>
BLA::Matrix<M,1> extractSub(const BLA::Matrix<N,1> &x, const std::array<uint8_t, M> &inds) {
    BLA::Matrix<M, 1> sub;

    for(int i=0; i < M; i++) {
        sub(i) = x(inds[i]);
    }

    return sub;
}

BLA::Matrix<3,3> quat2rot(const BLA::Matrix<4,1>& q) {
    float w = q(0), x = q(1), y = q(2), z = q(3);

    BLA::Matrix<3,3> R;
    R(0,0) = 1 - 2*(y*y + z*z);
    R(0,1) = 2*(x*y - z*w);
    R(0,2) = 2*(x*z + y*w);

    R(1,0) = 2*(x*y + z*w);
    R(1,1) = 1 - 2*(x*x + z*z);
    R(1,2) = 2*(y*z - x*w);

    R(2,0) = 2*(x*z - y*w);
    R(2,1) = 2*(y*z + x*w);
    R(2,2) = 1 - 2*(x*x + y*y);

    return R;
}