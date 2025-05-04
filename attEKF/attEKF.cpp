#include "attEkf.h"

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

    P_min = P;

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

void AttStateEstimator::init(BLA::Matrix<13,1> x_0, float dt) {
    this->dt = dt;

    this->x     = x_0;
    this->x_min = x_0;
}

void AttStateEstimator::onLoop(Context &ctx) {
    // Read data from sensors and convert values
    
    float gyrX = ctx.mag.getData().gyrX;
    float gyrY = ctx.mag.getData().gyrY;
    float gyrZ = ctx.mag.getData().gyrZ;

    float aclX = ctx.mag.getData().accelX;
    float aclY = ctx.mag.getData().accelY;
    float aclZ = ctx.mag.getData().accelZ;

    float magX = ctx.mag.getData().magX;
    float magY = ctx.mag.getData().magY;
    float magZ = ctx.mag.getData().magZ;

    BLA::Matrix<3,1> u = {gyrX, gyrY, gyrZ};   // [dps]
    BLA::Matrix<3,1> a_b = {aclX, aclY, aclZ}; // [g]
    BLA::Matrix<3,1> m_b = {magX, magY, magZ}; // [uT]

    // Update u_prev if first iteration
    if(!hasPassedGo) {
        u_prev = u;

        hasPassedGo = true;
    }

    x_min = propRK4(u);

    // Measurement Jacobian Matrix
    BLA::Matrix<13,13> F = predictionJacobian(u);

    // Discretize Measurement
    BLA::Matrix<13,13> phi = I_13 + F * dt;

    // Predict Error Covariance
    P_min = phi * F * BLA::MatrixTranspose<BLA::Matrix<13,13>>(phi) + Q;

    // ===== IF ON PAD ===== TODO
    applyGravUpdate(x_min, a_b);

    // ===== ALWAYS =====
    applyMagUpdate(x, m_b);
    // APPLY MAG UPDATE

    // Update previous gyro reading
    u_prev = u;
}

BLA::Matrix<13,1> AttStateEstimator::propRK4(BLA::Matrix<3,1> u) {

    BLA::Matrix<3,1> u_k1  = u_prev;
    BLA::Matrix<3,1> u_k   = u;
    BLA::Matrix<3,1> u_k12 = 0.5f * (u_k1 + u_k);

    BLA::Matrix<13,1> k1 = dt * predictionFunction(x, u);
    BLA::Matrix<13,1> k2 = dt * predictionFunction(x + k1*0.5f, u_k12);
    BLA::Matrix<13,1> k3 = dt * predictionFunction(x + k2*0.5f, u_k12);
    BLA::Matrix<13,1> k4 = dt * predictionFunction(x + k3, u_k);

    x_min = x + (k1*(1.0f/6.0f) + k2*(1.0f/3.0f) + k3*(1.0f/3.0f) + k4*(1.0f/6.0f));

    // Force Normalize Unit Quaternion
    BLA::Matrix<4,1> quat = {
        x_min(AttKFInds::q_w),
        x_min(AttKFInds::q_x),
        x_min(AttKFInds::q_y),
        x_min(AttKFInds::q_z)
    };

    quat = quat / BLA::Norm(quat);

    x_min(AttKFInds::q_w) = quat(0);
    x_min(AttKFInds::q_x) = quat(1);
    x_min(AttKFInds::q_y) = quat(2);
    x_min(AttKFInds::q_z) = quat(3);

    return x_min;
}

BLA::Matrix<13,1> AttStateEstimator::predictionFunction(BLA::Matrix<13,1> x, BLA::Matrix<3,1> u) {
    float p = u(0) - x(AttKFInds::gb_x);
    float q = u(1) - x(AttKFInds::gb_y);
    float r = u(2) - x(AttKFInds::gb_z);

    BLA::Matrix<4,3> quatMat = {
        -x(1), -x(2), -x(3),
         x(0), -x(3),  x(2),
         x(3),  x(0), -x(1), 
        -x(2),  x(1),  x(0), 
    };
    
    BLA::Matrix<3,1> gyr = {p, q, r};

    BLA::Matrix<4, 1> f_q = (quatMat * gyr) * 0.5f;

    // Assume constant bias
    BLA::Matrix<13,1> f = {
        f_q(0), f_q(1), f_q(2), f_q(3), 0, 0, 0, 0, 0, 0, 0, 0, 0
    };

    return f;
}

BLA::Matrix<13,13> AttStateEstimator::predictionJacobian(BLA::Matrix<3,1> u) {
    float gbx = x(AttKFInds::gb_x);
    float gby = x(AttKFInds::gb_y);
    float gbz = x(AttKFInds::gb_z);
    
    float p = u(0) - gbx;
    float q = u(1) - gby;
    float r = u(2) - gbz;

    float qw = x(AttKFInds::q_w);
    float qx = x(AttKFInds::q_x);
    float qy = x(AttKFInds::q_y);
    float qz = x(AttKFInds::q_z);

    BLA::Matrix<13,13> F = {
        0, 0.5*gbx - 0.5*p, 0.5*gby - 0.5*q, 0.5*gbz - 0.5*r, 0.5*qx, 0.5*qy, 0.5*qz, 0, 0, 0, 0, 0, 0,
        0.5*p - 0.5*gbx, 0, 0.5*r - 0.5*gbz, 0.5*gby - 0.5*q, -0.5*qw, 0.5*qz, -0.5*qy, 0, 0, 0, 0, 0, 0,
        0.5*q - 0.5*gby, 0.5*gbz - 0.5*r, 0, 0.5*p - 0.5*gbx, -0.5*qz, -0.5*qw, 0.5*qx, 0, 0, 0, 0, 0, 0,
        0.5*r - 0.5*gbz, 0.5*q - 0.5*gby, 0.5*gbx - 0.5*p, 0, 0.5*qy, -0.5*qx, -0.5*qw, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    };

    return F;
}

void AttStateEstimator::applyGravUpdate(BLA::Matrix<13,1> &x, BLA::Matrix<3,1> a_b) {
    BLA::Matrix<3,1> G_NED = {0, 0, -g};

    BLA::Matrix<4,1> q = {
        x(AttKFInds::q_w),
        x(AttKFInds::q_x),
        x(AttKFInds::q_y),
        x(AttKFInds::q_z)
    };

    BLA::Matrix<3,3> R_TB = quat2rot(q);

    BLA::Matrix<3,1> bias = {
        x(AttKFInds::ab_x),
        x(AttKFInds::ab_y),
        x(AttKFInds::ab_z)
    };
    
    BLA::Matrix<3,1> h_grav = BLA::MatrixTranspose<BLA::Matrix<3,3>>(R_TB)* G_NED + bias;

    BLA::Matrix<3,1> z_grav = a_b - bias;

    float qw = x(AttKFInds::q_w);
    float qx = x(AttKFInds::q_x);
    float qy = x(AttKFInds::q_y);
    float qz = x(AttKFInds::q_z);

    float abx = x(AttKFInds::ab_x);
    float aby = x(AttKFInds::ab_y);
    float abz = x(AttKFInds::ab_z);

    BLA::Matrix<3, 13> H_grav = {
         2*g*qy, -2*g*qz,  2*g*qw, -2*g*qx, 0, 0, 0, 1, 0, 0, 0, 0, 0,
        -2*g*qx, -2*g*qw, -2*g*qz, -2*g*qy, 0, 0, 0, 0, 1, 0, 0, 0, 0,
        -4*g*qw,  0,       0,      -4*g*qz, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    };

    BLA::Matrix<3,3> S = H_grav * P_min * BLA::MatrixTranspose<BLA::Matrix<3,13>>(H_grav) + R_grav;

    BLA::Matrix<13,3> K = P_min * BLA::MatrixTranspose<BLA::Matrix<3,13>>(H_grav) * BLA::Inverse(S);

    this->x = x + K * (z_grav - h_grav);

    P = (I_13 - K * H_grav) * P_min;
}

void AttStateEstimator::applyMagUpdate(BLA::Matrix<13,1> &x, BLA::Matrix<3,1> m_b) {
    BLA::Matrix<3,1> B_NED = {22.0f, 5.0f, -45.0f}; // [uT]

    float B_N = B_NED(0);
    float B_E = B_NED(1);
    float B_D = B_NED(2);

    BLA::Matrix<4,1> q = {
        x(AttKFInds::q_w),
        x(AttKFInds::q_x),
        x(AttKFInds::q_y),
        x(AttKFInds::q_z)
    };

    BLA::Matrix<3,3> R_TB = quat2rot(q);

    BLA::Matrix<3,1> bias = {
        x(AttKFInds::mb_x),
        x(AttKFInds::mb_y),
        x(AttKFInds::mb_z)
    };

    float qw = x(AttKFInds::q_w);
    float qx = x(AttKFInds::q_x);
    float qy = x(AttKFInds::q_y);
    float qz = x(AttKFInds::q_z);

    float mbx = x(AttKFInds::mb_x);
    float mby = x(AttKFInds::mb_y);
    float mbz = x(AttKFInds::mb_z);

    // BLA::Matrix<3,1> h_mag = BLA::MatrixTranspose<R_TB>() * B_NED + bias;
    BLA::Matrix<3,1> h_mag = BLA::MatrixTranspose<BLA::Matrix<3,3>>(R_TB) * B_NED + bias;
    BLA::Matrix<3,1> z_mag = m_b - bias;

    BLA::Matrix<3,13> H_mag = {
   
        2*B_E*qz - 2*B_D*qy,                       
        2*B_D*qz + 2*B_E*qy,                         
        2*B_E*qx - 2*B_D*qw - 4*B_N*qy,              
        2*B_D*qx + 2*B_E*qw - 4*B_N*qz,           
        0, 0, 0, 0, 0, 0,
        1, 0, 0,                                      
    
        2*B_D*qx - 2*B_N*qz,                      
        2*B_D*qw - 4*B_E*qx + 2*B_N*qy,             
        2*B_D*qz + 2*B_N*qx,                          
        2*B_D*qy - 4*B_E*qz - 2*B_N*qw,               
        0, 0, 0, 0, 0, 0,
        0, 1, 0,                                     
    
        2*B_N*qy - 2*B_E*qx,                         
        2*B_N*qz - 2*B_E*qw - 4*B_D*qx,               
        2*B_E*qz - 4*B_D*qy + 2*B_N*qw,              
        2*B_E*qy + 2*B_N*qx,                        
        0, 0, 0, 0, 0, 0,
        0, 0, 1                                     
    };

    BLA::Matrix<3,3> S = H_mag * P_min * BLA::MatrixTranspose<BLA::Matrix<3,13>>(H_mag) + R_mag;

    BLA::Matrix<13,3> K = P_min * BLA::MatrixTranspose<BLA::Matrix<3,13>>(H_mag) * BLA::Inverse(S);

    this->x = x + K * (z_mag - h_mag);

    P = (I_13 - K * H_mag) * P_min;
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
