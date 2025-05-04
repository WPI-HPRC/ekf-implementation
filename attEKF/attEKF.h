#pragma once

#include "Arduino.h"
#include "BasicLinearAlgebra.h"

#include "../sensorConst.h"

using namespace BLA;

struct AttKFInds {
    static constexpr uint8_t q_w = 0;
    static constexpr uint8_t q_x = 1;
    static constexpr uint8_t q_y = 2;
    static constexpr uint8_t q_z = 3;
    static constexpr std::array<uint8_t, 4> quat      = {q_w, q_x, q_y, q_z};

    static constexpr uint8_t gb_x = 4;
    static constexpr uint8_t gb_y = 5;
    static constexpr uint8_t gb_z = 6;
    static constexpr std::array<uint8_t, 3> gyroBias  = {gb_x, gb_y, gb_z};

    static constexpr uint8_t ab_x = 7;
    static constexpr uint8_t ab_y = 8;
    static constexpr uint8_t ab_z = 9;
    static constexpr std::array<uint8_t, 3> accelBias = {ab_x, ab_y, ab_z};
};

class AttStateEstimator {

    public:

    AttStateEstimator() = default;

    void init(BLA::Matrix<10,1> x_0, float dt);

    void onLoop(Utility::TelemPacket sensorPacket);

    private:

    // Prediction Method
    BLA::Matrix<10,1> predictionFunction(BLA::Matrix<10,1> x, BLA::Matrix<3,1> u);
    BLA::Matrix<10,10> predictionJacobian(BLA::Matrix<3,1> u);

    void updateFunction_Mag(BLA::Matrix<3,1> z);
    void updateJacobian_Mag(BLA::Matrix<3,1> z);

    BLA::Matrix<10,1> propRK4(BLA::Matrix<3,1> u);

    float dt = 0.0f;

    // State Vector
    BLA::Matrix<10,1> x_min;

    BLA::Matrix<10,1> x;

    // Error Covariance
    BLA::Matrix<10,10> P;

    BLA::Matrix<10,10> P_min;

    // Process Noise Covariance
    BLA::Matrix<10,10> Q;

    // Previous control input
    BLA::Matrix<3,1> u_prev = {0,0,0,0};

    uint32_t stateIter = 0;
};