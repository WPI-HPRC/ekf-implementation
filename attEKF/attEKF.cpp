#include "attEKF.h"

AttStateEstimator::AttStateEstimator() {
    // Initialize Error Covariance
    for(uint8_t idx : AttKFInds::quat) {
        P(idx, idx) = asm330_const.gyroXYZ_var;
    }
    for(uint8_t idx : AttKFInds::gyroBias) {
        P(idx,idx) = asm330_const.gyroXYZ_var;
    }

    memcpy(P_min, P);

    // Initialize Process Noise
    for(uint8_t idx : AttKFInds::quat) { 
        Q(idx, idx) = asm330_const.quat_var;
    }
    for(uint8_t idx : AttKFInds::gyroBias) {
        Q(idx, idx) = 0.001;
    }
}

void AttStateEstimator::init(BLA::Matrix<10,1> x_0, float dt) {
    this->dt = dt;

    this->x     = x_0;
    this->x_min = x_0;
}

void AttStateEstimator::onLoop(Utility::TelemPacket sensorPacket) {
    float gyrX = 0.0f;
    float gyrY = 0.0f;
    float gyrZ = 0.0f;

    float magX = 0.0f;
    float magY = 0.0f;
    float magZ = 0.0f;

    BLA::Matrix<3,1> u = {gyrX, gyrY, gyrZ};

    // Update u_prev
    if(stateIter == 0) {
        u_prev = u;
    }

    x_min = propRK4(BLA::Matrix<3,1> u);

    // Measurement Jacobian Matrix
    BLA::Matrix<10,10> F = predictionJacobian(u);

    // Discretize Measurement
    BLA::Matrix<10,10> phi = BLA::Identity<3>() + F * dt;

    // Predict Error Covariance
    P_min = phi * F * phi.T + Q;

    // ===== IF ON PAD ===== TODO
    // APPLY GRAV UPDATE

    // ===== ALWAYS =====
    // APPLY MAG UPDATE
}

BLA::Matrix<10,1> AttStateEstimator::propRK4(BLA::Matrix<3,1> u) {

    BLA::Matrix<3,1> u_k1  = u_prev;
    BLA::Matrix<3,1> u_k   = u;
    BLA::Matrix<3,1> u_k12 = 0.5f * (u_k1 + u_k);

    BLA::Matrix<10,1> k1 = dt * predictionFunction(x, u);
    BLA::Matrix<10,1> k2 = dt * predictionFunction(x + 0.5*k1, u_k12);
    BLA::Matrix<10,1> k3 = dt * predictionFunction(x + 0.5*k2, u_k12);
    BLA::Matrix<10,1> k4 = dt * predictionFunction(x + k3, u_k);

    x_min = x + ((1/6)*k1 + (1/3)*k2 + (1/3)*k3 + (1/6)*k4);

    // Force Normalize Unit Quaternion
    quat = x_min[AttKFInds::quat.begin(), AttKFInds::quat.end()];
    x_min[AttKFInds::quat.begin(), AttKFInds::quat.end()] = quat / BLA::norm(quat);

    return x_min;
}

BLA::Matrix<10,1> predictionFunction(BLA::Matrix<10,1> x, BLA::Matrix<3,1> u) {
    float p = u[0] - x[AttKFInds::gb_x];
    float q = u[1] - x[AttKFInds::gb_y];
    float r = u[2] - x[AttKFInds::gb_z];

    BLA::Matrix<4,3> quatMat = {
        -x[1], -x[2], -x[3],
         x[0], -x[3],  x[2],
         x[3],  x[0], -x[1], 
        -x[2],  x[1],  x[0], 
    };
    
    BLA::Matrix<3,1> gyr = {p, q, r};

    BLA::Matrix<4, 1> f_q = (multiplicand * gyr) * 0.5f;

    // Assume constant bias
    BLA::Matrix<7,1> f = {
        f_q[0], f_q[1], f_q[2], f_q[3], 0, 0, 0
    };

    return f;
}

BLA::Matrix<10,10> predictionJacobian(BLA::Matrix<3,1> u) {
    float gbx = x[AttKFInds::gb_x];
    float gby = x[AttKFInds::gb_y];
    float gbz = x[AttKFInds::gb_z];
    
    float p = u[0] - gbx;
    float q = u[1] - gby;
    float r = u[2] - gbz;

    float qw = x[AttKFInds::q_w];
    float qx = x[AttKFInds::q_x];
    float qy = x[AttKFInds::q_y];
    float qz = x[AttKFInds::q_z];

    BLA::Matrix<10,10> F = {
        0, 0.5*gbx - 0.5*p, 0.5*gby - 0.5*q, 0.5*gbz - 0.5*r, 0.5*qx, 0.5*qy, 0.5*qz, 0, 0, 0,
        0.5*p - 0.5*gbx, 0, 0.5*r - 0.5*gbz, 0.5*gby - 0.5*q, -0.5*qw, 0.5*qz, -0.5*qy, 0, 0, 0,
        0.5*q - 0.5*gby, 0.5*gbz - 0.5*r, 0, 0.5*p - 0.5*gbx, -0.5*qz, -0.5*qw, 0.5*qx, 0, 0, 0,
        0.5*r - 0.5*gbz, 0.5*q - 0.5*gby, 0.5*gbx - 0.5*p, 0, 0.5*qy, -0.5*qx, -0.5*qw, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    };

    return F;
};