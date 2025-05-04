#include "attEKF.h"

AttStateEstimator::AttStateEstimator() {
    // Initialize Error Covariance
    for(uint8_t idx : AttKFInds::quat) {
        P(idx, idx) = icm20948_const.gyroXYZ_var;
    }
    for(uint8_t idx : AttKFInds::gyroBias) {
        P(idx,idx) = icm20948_const.gyroXYZ_var;
    }
    
    P(AttKFInds::accelBias[0], AttKFInds::accelBias[0]) = icm20948_const.accelXY_var;
    P(AttKFInds::accelBias[1], AttKFInds::accelBias[1]) = icm20948_const.accelXY_var;
    P(AttKFInds::accelBias[2], AttKFInds::accelBias[2]) = icm20948_const.accelZ_var;

    for(uint8_t idx : AttKFInds::magBias) {
        P(idx, idx) = icm20948_const.magXYZ_var;
    }

    memcpy(P_min, P);

    // Initialize Process Noise
    for(uint8_t idx : AttKFInds::quat) { 
        Q(idx, idx) = icm20948_const.quatVar;
    }
    for(uint8_t idx : AttKFInds::gyroBias) {
        Q(idx, idx) = 0.001;
    }
    for(uint8_t idx : AttKFInds::accelBias) {
        Q(idx, idx) = 0.001;
    }
    for(uint8_t idx : AttKFInds::magBias) {
        Q(idx, idx) = 0.001;
    }
}

void AttStateEstimator::init(BLA::Matrix<10,1> x_0, float dt) {
    this->dt = dt;

    this->x     = x_0;
    this->x_min = x_0;
}

void AttStateEstimator::onLoop(Utility::TelemPacket sensorPacket) {
    // Read data from sensors and convert values
    float gyrX = 0.0f;
    float gyrY = 0.0f;
    float gyrZ = 0.0f;

    float magX = 0.0f;
    float magY = 0.0f;
    float magZ = 0.0f;

    BLA::Matrix<3,1> u = {gyrX, gyrY, gyrZ};

    // Update u_prev if first iteration
    if(!hasPassedGo) {
        u_prev = u;

        hasPassedGo = true;
    }

    x_min = propRK4(BLA::Matrix<3,1> u);

    // Measurement Jacobian Matrix
    BLA::Matrix<10,10> F = predictionJacobian(u);

    // Discretize Measurement
    BLA::Matrix<10,10> phi = BLA::Identity<3>() + F * dt;

    // Predict Error Covariance
    P_min = phi * F * phi.T + Q;

    // ===== IF ON PAD ===== TODO
    applyGravUpdate();

    // ===== ALWAYS =====
    // APPLY MAG UPDATE

    // Update previous gyro reading
    u_prev = u;
    stateIter++;
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
    BLA::Matrix<4,1> quat = x_min[AttKFInds::quat.begin(), AttKFInds::quat.end()];
    x_min[AttKFInds::quat.begin(), AttKFInds::quat.end()] = quat / BLA::norm(quat);

    return x_min;
}

BLA::Matrix<10,1> AttStateEstimator::predictionFunction(BLA::Matrix<10,1> x, BLA::Matrix<3,1> u) {
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

BLA::Matrix<10,10> AttStateEstimator::predictionJacobian(BLA::Matrix<3,1> u) {
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
}

void AttStateEstimator::applyGravUpdate(BLA::Matrix<3,1> a_b) {
    BLA::Matrix<3,1> G_NED = {0, 0, -g};

    auto q = extractSub(x_min, AttKFInds::quat);

    BLA::Matrix<3,3> R_TB = quat2rot(q);

    BLA::Matrix<3,1> bias = extractSub(x_min, AttKFInds::accelBias);
    
    BLA::Matrix<3,1> h_grav = R_TB.T * G_NED + bias;

    BLA::Matrix<3,1> z_grav = a_b - bias;

    float qw = x[AttKFInds::q_w];
    float qx = x[AttKFInds::q_x];
    float qy = x[AttKFInds::q_y];
    float qz = x[AttKFInds::q_z];

    float abx = x[AttKFInds::ab_x];
    float aby = x[AttKFInds::ab_y];
    float abz = x[AttKFInds::ab_z];

    BLA::Matrix<3, 10> H_grav = {
         2*g*qy, -2*g*qz,  2*g*qw, -2*g*qx, 0, 0, 0, 1, 0, 0,
        -2*g*qx, -2*g*qw, -2*g*qz, -2*g*qy, 0, 0, 0, 0, 1, 0,
        -4*g*qw,  0,       0,      -4*g*qz, 0, 0, 0, 0, 0, 1
    };

    R_grav = BLA::Identity<3>() * asm330_const.accelXYZ_var;

    S = H * P_min * H.T + R_grav;
    K = P_min * H.T / S;

    x = x_min + K * (z_grav-h_grav);

    P = (BLA::Identity<10>() - K*H) * P_min;

}

void AttStateEstimator::applyMagUpdate(BLA::Matrix<3,1> m_b) {
    
}