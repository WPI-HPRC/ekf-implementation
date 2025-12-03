#include "QuaternionUtils.h"
#include "BasicLinearAlgebra.h"
#include <cmath>
#include <iostream>

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

BLA::Matrix<3, 1> QuaternionUtils::getUpVector(const BLA::Matrix<3, 3> &rot) {
    return BLA::Matrix<3, 1>(rot(0, 1), rot(1, 1), rot(2, 1));
}

BLA::Matrix<3, 1> QuaternionUtils::getForwardVector(const BLA::Matrix<3, 3> &rot) {
    return BLA::Matrix<3, 1>(rot(0, 2), rot(1, 2), rot(2, 2));
}

BLA::Matrix<3,1> QuaternionUtils::getRightVector(const BLA::Matrix<3,3> &rot) {
    return BLA::Matrix<3,1>(rot(0,0), rot(1,0), rot(2,0));
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

BLA::Matrix<4, 1> QuaternionUtils::smallAngleRotVec2Quat(const BLA::Matrix<3, 1> &vec) {
    BLA::Matrix<4, 1> quat;
    quat(0, 0) = 1.0;
    quat(1, 0) = 0.5 * vec(0, 0);
    quat(2, 0) = 0.5 * vec(1, 0);
    quat(3, 0) = 0.5 * vec(2, 0);

    return quat;
}

BLA::Matrix<4, 1> QuaternionUtils::dcm2quat(const BLA::Matrix<3, 3> &dcm) {
    // TODO
    BLA::Matrix<4, 1> tmp_q = {1, 0, 0, 0};
    return tmp_q;
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

BLA::Matrix<3, 3> QuaternionUtils::dcm_ned2ecef(const BLA::Matrix<3, 1> &lla) {
    float pi = 3.141592653; // TODO declare this somewhere else but idk c++
    float lat_rads = lla(0) * (pi / 180.0);
    float lon_rads = lla(1) * (pi / 180.0);
    BLA::Matrix<3, 3> R_ET = {
        -1.0f * sin(lat_rads) * cos(lon_rads), -1.0f * sin(lon_rads), -1.0f * cos(lat_rads) * cos(lon_rads),
        -1.0f * sin(lat_rads) * sin(lon_rads), cos(lon_rads), -1.0f * cos(lat_rads) * sin(lon_rads),
        cos(lat_rads), 0, -1.0 * sin(lat_rads)
    };
    return R_ET;
}

BLA::Matrix<3, 1> QuaternionUtils::ecef2nedVec(const BLA::Matrix<3, 1> &ecef_meas, const BLA::Matrix<3, 1> &launch_ecef, const BLA::Matrix<3, 3> &R_ET) {
    return ~R_ET * (ecef_meas - launch_ecef);

}

BLA::Matrix<4, 1> QuaternionUtils::quatConjugate(const BLA::Matrix<4, 1> &p){
    BLA::Matrix<4, 1> quat;
    quat(0, 0) = p(0);
    quat(1, 0) = -1 * p(1);
    quat(2, 0) = -1 * p(2);
    quat(3, 0) = -1 * p(3);

    return quat;
}

BLA::Matrix<3, 1> QuaternionUtils::g_i_ecef(const BLA::Matrix<3, 3> dcm_ned2ecef) {
    // TODO one day put in the world model
    BLA::Matrix<3, 1> grav_ned = {0, 0, 9.8};
    return ~dcm_ned2ecef * grav_ned;
}

BLA::Matrix<3, 1> QuaternionUtils::m_i_ecef(const BLA::Matrix<3, 3> dcm_ned2ecef) {
    // TODO one day put in the world model. School vals for now
    BLA::Matrix<3, 1> mag_ned = {19.983111, -4.8716, 46.9986145};
    return ~dcm_ned2ecef * mag_ned;
}

BLA::Matrix<3, 1> QuaternionUtils::normal_i_ecef(const BLA::Matrix<3, 3> dcm_ned2ecef) {
    // TODO one day put in the world model
    BLA::Matrix<3, 1> normal_ned = {0, 0, -9.8};
    return ~dcm_ned2ecef * normal_ned;
}

BLA::Matrix<3, 3> QuaternionUtils::vecs2mat(const BLA::Matrix<3, 1> v1, const BLA::Matrix<3, 1> v2, const BLA::Matrix<3, 1> v3) {
    BLA::Matrix<3, 3> ret = {v1(0, 0), v2(0), v3(0, 0),
                            v1(1, 0), v2(1, 0), v3(1, 0),
                            v1(2, 0), v2(2, 0), v3(2, 0)};

    return ret;
}



BLA::Matrix<4, 1> QuaternionUtils::triad_BE(const BLA::Matrix<3, 1> v1_b, const BLA::Matrix<3, 1> v2_b, const BLA::Matrix<3, 1> v1_i, const BLA::Matrix<3, 1> v2_i) {
    // 1 is important, 2 is secondary
    BLA::Matrix<3, 1> v1_b_norm = v1_b / BLA::Norm(v1_b);
    BLA::Matrix<3, 1> v2_b_norm = v1_b / BLA::Norm(v2_b);
    BLA::Matrix<3, 1> v1_i_norm = v1_b / BLA::Norm(v1_i);
    BLA::Matrix<3, 1> v2_i_norm = v1_b / BLA::Norm(v2_i);

    // Inertial
    BLA::Matrix<3, 1> q_r = v1_i;
    BLA::Matrix<3, 1> r_r = BLA::CrossProduct(v1_i, v2_i) / BLA::Norm(BLA::CrossProduct(v1_i, v2_i));
    BLA::Matrix<3, 1> s_r = BLA::CrossProduct(q_r, r_r);

    BLA::Matrix<3, 3> M_r = vecs2mat(q_r, r_r, s_r);


    // Body
    BLA::Matrix<3, 1> q_b = v1_b;
    BLA::Matrix<3, 1> r_b = BLA::CrossProduct(v1_b, v2_b) / BLA::Norm(BLA::CrossProduct(v1_b, v2_b));
    BLA::Matrix<3, 1> s_b = BLA::CrossProduct(q_b, r_b);

    BLA::Matrix<3, 3> M_b = vecs2mat(q_b, r_b, s_b);

    BLA::Matrix<3, 3> R_BE = ~(M_r * ~M_b);

    return dcm2quat(R_BE);

}

BLA::Matrix<3,1> QuaternionUtils::lla2ecef(BLA::Matrix<3,1> lla) {
    float pi = 3.141592653589; // TODO move this somewhere better (idk c++)
    float a = 6378137.0;                // WGS-84 semi-major axis
    float f = 1.0 / 298.257223563;      // flattening
    float e2 = f * (2.0f - f);             // eccentricity squared

    float lat = lla(0) * pi / 180.0; // Convert to radians 
    float lon = lla(1) * pi / 180.0; // Convert to radians 
    float alt = lla(2); 

    // Convert reference lla to ecef first 
    float N = a / std::sqrt(1 - e2 * std::pow(std::sin(lat), 2));

    float x = (N + alt) * std::cos(lat) * std::cos(lon);
    float y = (N + alt) * std::cos(lat) * std::sin(lon);
    float z = ((1 - e2) * N + alt) * std::sin(lat);

    BLA::Matrix<3,1> ecef_coords = {x,y,z}; 

    return ecef_coords; 

}

BLA::Matrix<3, 1> QuaternionUtils::ecef2lla(BLA::Matrix<3, 1> ecef) {
    // Follows this kid idk if right, gotta test, but I'm not writing this bs myself
    // I think this is's impl not super precise, but not that deep (I think)
    // https://github.com/kvenkman/ecef2lla/blob/master/ecef2lla.py

    float pi = 3.141592653589;
	float a = 6378137.0f;
	float a_sq = a * a;
	float e = 8.181919084261345e-2;
	float e_sq = 6.69437999014e-3;

	float f = 1.0f/298.257223563;
	float b = a*(1.0-f);

    float r = std::sqrt(ecef(0) * ecef(0) + ecef(1) * ecef(1));
    float ep_sq = (a * a - b * b) / (b * b);
    float ee = (a * a - b * b);
    f = (54 * b * b) * (ecef(2) * ecef(2));
	float g = r * r + (1 - e_sq) * (ecef(2) * ecef(2)) - e_sq * ee * 2.0f;
    float c = (e_sq * e_sq) * f * r * r / (g * g * g);
    float s = std::pow((1.0 + c + std::sqrt(c * c + 2 * c)), (1.0f / 3.0f));
    float p = f / (3.0f * (g * g) * std::pow(s + (1.0 / s) + 1.0, 2.0f));
	float q = std::sqrt(1.0f + 2.0f * p * std::pow(e_sq, 2.0f));
	// float r_0 = 
    // TODO got lazy, gotta finish this tho
	// r_0 = -(p*e_sq*r)/(1+q) + np.sqrt(0.5*(a**2)*(1+(1./q)) - p*(z**2)*(1-e_sq)/(q*(1+q)) - 0.5*p*(r**2))
	// u = np.sqrt((r - e_sq*r_0)**2 + z**2)
	// v = np.sqrt((r - e_sq*r_0)**2 + (1 - e_sq)*z**2)
	// z_0 = (b**2)*z/(a*v)
	// h = u*(1 - b**2/(a*v))
	// phi = np.arctan((z + ep_sq*z_0)/r)
	// lambd = np.arctan2(y, x)

    float phi = 0.0f;
    float lambd = 0.0f;
    float h = 0.0f;
    BLA::Matrix<3, 1> lla = {phi*180.0f/pi, lambd*180.0/pi, h};
	return lla;
}




