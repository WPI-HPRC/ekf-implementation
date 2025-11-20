#include "QuaternionUtils.h"
#include "BasicLinearAlgebra.h"
#include <cmath>

BLA::Matrix<3, 3> QuaternionUtils::quatToRot(const BLA::Matrix<4, 1> &quat) {
    float w = quat(0);
    float x = quat(1);
    float y = quat(2);
    float z = quat(3);

    BLA::Matrix<3, 3> rot;

    // Row1
    rot(0, 0) = 1 - 2 * y * y - 2 * z * z;
    rot(0, 1) = 2 * x * y - 2 * w * z;
    rot(0, 2) = 2 * x * z + 2 * w * y;

    // Row2
    rot(1, 0) = 2 * x * y + 2 * w * z;
    rot(1, 1) = 1 - 2 * x * x - 2 * z * z;
    rot(1, 2) = 2 * y * z - 2 * w * x;

    // Row3
    rot(2, 0) = 2 * x * z - 2 * w * y;
    rot(2, 1) = 2 * y * z + 2 * w * x;
    rot(2, 2) = 1 - 2 * x * x - 2 * y * y;

    return rot;
}

BLA::Matrix<3, 3> QuaternionUtils::quat2DCM(const BLA::Matrix<4, 1> &quat) {
    float w = quat(0);
    float x = quat(1);
    float y = quat(2);
    float z = quat(3);

    BLA::Matrix<3, 3> rot;

    // Row1
    rot(0, 0) = w * w + x * x - y * y - z * z;
    rot(0, 1) = 2 * x * y + 2 * w * z;
    rot(0, 2) = 2 * x * z - 2 * w * y;

    // Row2
    rot(1, 0) = 2 * x * y - 2 * w * z;
    rot(1, 1) = w * w - x * x + y * y - z * z;
    rot(1, 2) = 2 * y * z + 2 * w * x;

    // Row3
    rot(2, 0) = 2 * x * z + 2 * w * y;
    rot(2, 1) = 2 * y * z - 2 * w * x;
    rot(2, 2) = w * w - x * x - y * y + z * z;

    return rot;
}

BLA::Matrix<3, 1> QuaternionUtils::getUpVector(const BLA::Matrix<3, 3> &rot) {
    return BLA::Matrix<3, 1>(rot(0, 1), rot(1, 1), rot(2, 1));
}

BLA::Matrix<3, 1> QuaternionUtils::getForwardVector(const BLA::Matrix<3, 3> &rot) {
    return BLA::Matrix<3, 1>(rot(0, 2), rot(1, 2), rot(2, 2));
}

BLA::Matrix<3,1> QuaternionUtils::getRightVector(const BLA::Matrix<3,3> &rot) {
    return BLA::Matrix<3,1>(rot(0,0), rot(1,0), rot(2,0));
}

BLA::Matrix<3, 3> QuaternionUtils::skewSymmetric(const BLA::Matrix<3, 1> &vec) {
    BLA::Matrix<3, 3> mat;

    mat(0, 0) = 0;
    mat(0, 1) = -1 * vec(2, 0);
    mat(0, 2) = vec(1, 0);

    mat(1, 0) = vec(2, 0);
    mat(1, 1) = 0;
    mat(1, 2) = -1 * vec(0, 0);

    mat(2, 0) = -1 * vec(1, 0);
    mat(2, 1) = vec(0, 0);
    mat(2, 2) = 0;

    return mat;

}


BLA::Matrix<4, 1> QuaternionUtils::rotVec2Quat(const BLA::Matrix<3, 1> &vec) {
    BLA::Matrix<4, 1> quat;

    float norm = BLA::Norm(vec);
    BLA::Matrix<3, 1> vec_rot_normed = vec / norm;

    quat(0, 0) = cos(norm / 2);
    quat(1, 0) = vec_rot_normed(0, 0) * sin(norm / 2);
    quat(2, 0) = vec_rot_normed(1, 0) * sin(norm / 2);
    quat(3, 0) = vec_rot_normed(2, 0) * sin(norm / 2);

    return quat;
}

BLA::Matrix<4, 1> QuaternionUtils::quatMultiply(const BLA::Matrix<4, 1> &p, const BLA::Matrix<4, 1> &q) {
    BLA::Matrix<4, 1> quat;

    quat(0, 0) = p(0, 0) * q(0, 0) - p(1, 0) * q(1, 0) - p(2, 0) * q(2, 0) - p(3, 0) * q(3, 0);
    quat(1, 0) = p(0, 0) * q(1, 0) + p(1, 0) * q(0, 0) + p(2, 0) * q(3, 0) - p(3, 0) * q(2, 0);
    quat(2, 0) = p(0, 0) * q(2, 0)  - p(1, 0) * q(3, 0) + p(2, 0) * q(1, 0) + p(3, 0) * q(1, 0);
    quat(3, 0) = p(0, 0) * q(3, 0) + p(1, 0) * q(2, 0) - p(2, 0) * q(1, 0) + p(3, 0) * q(3, 0);


    return quat;
}

BLA::Matrix<3, 1> QuaternionUtils::lla2ecef(const BLA::Matrix<3, 1> &lla) {
    float pi = 3.141592;
    float lat_rad = lla(0) * pi / 180.0;
    float lon_rad = lla(1) * pi / 180.0;
    
    float a = 6378.0 * 1000.0;
    float b = 6357.0 * 1000.0;

    float e = std::sqrt(1.0 - (std::pow(b, 2) / std::pow(a, 2)));

    float N = a / std::sqrt(1.0 - std::pow(e, 2) * std::pow(sin(lat_rad), 2));

    float x = (N + lla(2)) * cos(lat_rad) * cos(lon_rad);
    float y = (N + lla(2)) * cos(lat_rad) * sin(lon_rad);
    float z = ((1 - std::pow(e, 2)) * N + lla(2)) * sin(lat_rad);

    BLA::Matrix<3, 1> ecef = {x, y, z};

    return ecef;


}

BLA::Matrix<3, 3> QuaternionUtils::dcm_ned2ecef(const BLA::Matrix<2, 1> &ll) {
    float pi = 3.141592653;
    float lat_rads = ll(0) * (pi / 180.0);
    float lon_rads = ll(1) * (pi / 180.0);
    BLA::Matrix<3, 3> R_ET = {
        -1.0 * sin(lat_rads) * cos(lon_rads), -1.0 * sin(lon_rads), -1.0 * cos(lat_rads) * cos(lon_rads),
        -1.0 * sin(lat_rads) * sin(lon_rads), cos(lon_rads), -1.0 * cos(lat_rads) * sin(lon_rads),
        cos(lat_rads), 0, -1.0 * sin(lat_rads)
    };
    return R_ET;
}

BLA::Matrix<3, 1> QuaternionUtils::ecef2ned(const BLA::Matrix<3, 1> &ecef_meas, const BLA::Matrix<3, 1> &launch_ecef, const BLA::Matrix<3, 3> &R_ET) {
    return ~R_ET * (ecef_meas - launch_ecef);

}



BLA::Matrix<4, 1> quatConjugate(const BLA::Matrix<4, 1> &p){
    BLA::Matrix<4, 1> quat;
    quat(0, 0) = p(0);
    quat(1, 0) = -1 * p(1);
    quat(2, 0) = -1 * p(2);
    quat(3, 0) = -1 * p(3);

    return quat;
}
