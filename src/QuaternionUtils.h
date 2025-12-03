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

    // Small angle rot vec to quat
    BLA::Matrix<4, 1> smallAngleRotVec2Quat(const BLA::Matrix<3, 1> &vec);

    BLA::Matrix<4, 1> dcm2quat(const BLA::Matrix<3, 3> &dcm);

    // Quaternion multiply
    BLA::Matrix<4, 1> quatMultiply(const BLA::Matrix<4, 1> &p, const BLA::Matrix<4, 1> &q);

    BLA::Matrix<3, 1> lla2ecef(const BLA::Matrix<3, 1> &lla);

    BLA::Matrix<3, 3> dcm_ned2ecef(const BLA::Matrix<3, 1> &lla);

    BLA::Matrix<3, 1> ecef2nedVec(const BLA::Matrix<3, 1> &ecef_meas, const BLA::Matrix<3, 1> &launch_ecef, const BLA::Matrix<3, 3> &R_ET);

    //Quaternion Conjugate
    BLA::Matrix<4, 1> quatConjugate(const BLA::Matrix<4, 1> &p);

    BLA::Matrix<3, 1> g_i_ecef(const BLA::Matrix<3, 3> dcm_ned2ecef);

    BLA::Matrix<3, 1> m_i_ecef(const BLA::Matrix<3, 3> dcm_ned2ecef);

    BLA::Matrix<3, 1> normal_i_ecef(const BLA::Matrix<3, 3> dcm_ned2ecef);

    BLA::Matrix<3, 3> vecs2mat(const BLA::Matrix<3, 1> v1, const BLA::Matrix<3, 1> v2, const BLA::Matrix<3, 1> v3);

    BLA::Matrix<4, 1> triad_BE(const BLA::Matrix<3, 1> v1_b, const BLA::Matrix<3, 1> v2_b, const BLA::Matrix<3, 1> v1_i, const BLA::Matrix<3, 1> v2_i);

    BLA::Matrix<3, 1> lla2ecef(BLA::Matrix<3,1> lla);

    BLA::Matrix<3, 1> ecef2lla(BLA::Matrix<3, 1> ecef);

    // template <size_t N, size_t M>
    // BLA::Matrix<M, 1> extractSub(const BLA::Matrix<N, 1> &x,
    //                             const std::array<uint8_t, M> &inds) {
    //     BLA::Matrix<M, 1> sub;
    //     for (int i = 0; i < M; i++) {
    //         sub(i) = x(inds[i]);
    //     }
    //     return sub;
    // }

    // template <size_t N>
    // BLA::Matrix<N, 1> extractDiag(const BLA::Matrix<N, N> &x) {
    //     BLA::Matrix<N, 1> diag;
    //     for (int i = 0; i < M; i++) {
    //         diag(i) = x(inds[i], indx[i]);
    //     }
    //     return diag;
    // }

    // template <size_t N>
    // float vecMax(const BLA::Matrix<N, 1> &x) {
    //     BLA::Matrix<N, 1> diag;
    //     for (int i = 0; i < M; i++) {
    //         diag(i) = x(inds[i], indx[i]);
    //     }
    //     return diag;
    //     // TODO fix
    // }
}
