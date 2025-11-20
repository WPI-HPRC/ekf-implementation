#pragma once

#include "BasicLinearAlgebra.h"

namespace QuaternionUtils {
    // Convert quaternion (w,x,y,z) to rotation matrix
    // quat SHOULD be normalized
    BLA::Matrix<3,3> quatToRot(const BLA::Matrix<4,1> &quat);

    // Quaternion to DCM
    BLA::Matrix<3, 3> quat2DCM(const BLA::Matrix<4, 1> &quat);

    // Get the up vector from the rotation matrix for the payload
    // In NED coordinates, this will be the second column of the rotation matrix
    BLA::Matrix<3,1> getUpVector(const BLA::Matrix<3,3> &rot);

    // Get the forward vector from the rotation matrix for the payload
    // NED coord == third col
    BLA::Matrix<3,1> getForwardVector(const BLA::Matrix<3,3> &rot);

    // Get the right vector from the rotation matrix for the payload
    // NED coord == first col
    BLA::Matrix<3,1> getRightVector(const BLA::Matrix<3,3> &rot);

    // Get skew symmetric of a 3x1 vector
    BLA::Matrix<3, 3> skewSymmetric(const BLA::Matrix<3, 1> &vec);

    // Rotation vector to quaternion
    BLA::Matrix<4, 1> rotVec2Quat(const BLA::Matrix<3, 1> &vec);

    // Quaternion multiply
    BLA::Matrix<4, 1> quatMultiply(const BLA::Matrix<4, 1> &p, const BLA::Matrix<4, 1> &q);

    BLA::Matrix<3, 1> lla2ecef(const BLA::Matrix<3, 1> &lla);

    BLA::Matrix<3, 3> dcm_ned2ecef(const BLA::Matrix<2, 1> &ll);

    BLA::Matrix<3, 1> ecef2ned(const BLA::Matrix<3, 1> &ecef_meas, const BLA::Matrix<3, 1> &launch_ecef, const BLA::Matrix<3, 3> &R_ET);

    //Quaternion Conjugate
    BLA::Matrix<4, 1> quatConjugate(const BLA::Matrix<4, 1> &p);

}
