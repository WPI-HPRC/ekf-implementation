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


// TODO redo
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

// TODO dont be lazy and write
BLA::Matrix<3, 3> QuaternionUtils::quat2DCMInv(const BLA::Matrix<4, 1> &quat) {
    // TODO one day rewrite it to make it more efficient
    return ~quat2DCM(quat);
}

// Works
BLA::Matrix<3, 3> QuaternionUtils::skewSymmetric(const BLA::Matrix<3, 1> &vec) {
    // Can rewrite as hstack
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

// Works
BLA::Matrix<4, 1> QuaternionUtils::rotVec2dQuat(const BLA::Matrix<3, 1> &vec) {
    BLA::Matrix<4, 1> quat;

    float norm = BLA::Norm(vec);
    BLA::Matrix<3, 1> vec_rot_normed = vec / norm;
    float norm_over_two = norm / 2.0f;

    quat(0, 0) = cos(norm_over_two);
    quat(1, 0) = vec_rot_normed(0, 0) * sin(norm_over_two);
    quat(2, 0) = vec_rot_normed(1, 0) * sin(norm_over_two);
    quat(3, 0) = vec_rot_normed(2, 0) * sin(norm_over_two);

    return quat;
}

// Works
BLA::Matrix<4, 1> QuaternionUtils::smallAnglerotVec2dQuat(const BLA::Matrix<3, 1> &vec) {
    BLA::Matrix<4, 1> quat;
    quat(0, 0) = 1.0;
    quat(1, 0) = 0.5 * vec(0, 0);
    quat(2, 0) = 0.5 * vec(1, 0);
    quat(3, 0) = 0.5 * vec(2, 0);

    return quat;
}

// Works
BLA::Matrix<4, 1> QuaternionUtils::dcm2quat(const BLA::Matrix<3, 3> &dcm) {
    BLA::Matrix<4, 1> q_sq;
    q_sq(0, 0) = 0.25f * (1.0f + dcm(0, 0) - dcm(1, 1) - dcm(2, 2));
    q_sq(1, 0) = 0.25f * (1.0f - dcm(0, 0) + dcm(1, 1) - dcm(2, 2));
    q_sq(2, 0) = 0.25f * (1.0f - dcm(0, 0) - dcm(1, 1) + dcm(2, 2));
    q_sq(3, 0) = 0.25f * (1.0f + dcm(0, 0) + dcm(1, 1) + dcm(2, 2));

    int max_ind = vecMaxInd(q_sq);
    BLA::Matrix<4, 1> q_ret;

    switch (max_ind) {
        case 0:
            q_ret(1, 0) = 4.0 * q_sq(0, 0);
            q_ret(2, 0) = dcm(0, 1) + dcm(1, 0);
            q_ret(3, 0) = dcm(2, 0) + dcm(0, 2);
            q_ret(0, 0) = dcm(1, 2) - dcm(2, 1);
            q_ret = q_ret / (4.0f * sqrt(q_sq(0, 0)));
        case 1:
            q_ret(1, 0) = dcm(0, 1) + dcm(1, 0);
            q_ret(2, 0) = 4.0 * q_sq(1, 0);
            q_ret(3, 0) = dcm(1, 2) + dcm(2, 1);
            q_ret(0, 0) = dcm(2, 0) - dcm(0, 2);
            q_ret = q_ret / (4.0f * sqrt(q_sq(1, 0)));
        case 2:
            q_ret(1, 0) = dcm(2, 0) + dcm(0, 2);
            q_ret(2, 0) = dcm(1, 2) + dcm(2, 1);
            q_ret(3, 0) = 4.0 * q_sq(2, 0);
            q_ret(0, 0) = dcm(0, 1) - dcm(1, 0);
            q_ret = q_ret / (4.0f * sqrt(q_sq(2, 0)));
        case 3:
            q_ret(1, 0) = dcm(1, 2) - dcm(2, 1);
            q_ret(2, 0) = dcm(2, 0) - dcm(0, 2);
            q_ret(3, 0) = dcm(0, 1) - dcm(1, 0);
            q_ret(0, 0) = 4.0 * q_sq(3, 0);
            q_ret = q_ret / (4.0f * sqrt(q_sq(3, 0)));
    }

    return q_ret;
}

// TODO test 
BLA::Matrix<4, 1> QuaternionUtils::quatMultiply(const BLA::Matrix<4, 1> &p, const BLA::Matrix<4, 1> &q) {
    BLA::Matrix<4, 1> quat;

    quat(0, 0) = p(0, 0) * q(0, 0) - p(1, 0) * q(1, 0) - p(2, 0) * q(2, 0) - p(3, 0) * q(3, 0);
    quat(1, 0) = p(0, 0) * q(1, 0) + p(1, 0) * q(0, 0) + p(2, 0) * q(3, 0) - p(3, 0) * q(2, 0);
    quat(2, 0) = p(0, 0) * q(2, 0)  - p(1, 0) * q(3, 0) + p(2, 0) * q(1, 0) + p(3, 0) * q(1, 0);
    quat(3, 0) = p(0, 0) * q(3, 0) + p(1, 0) * q(2, 0) - p(2, 0) * q(1, 0) + p(3, 0) * q(3, 0);

    // Should we always normalize here?
    return quat;

}

// Note: Not super high precision like I want, however unlikely to convert this way
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

// TODO test again
BLA::Matrix<3, 3> QuaternionUtils::dcm_ned2ecef(const BLA::Matrix<3, 1> &lla) {
    BLA::Matrix<3, 3> R_ET = {
        -1.0f * sind(lla(0)) * cosd(lla(1)), -1.0f * sind(lla(1)), -1.0f * cosd(lla(0)) * cosd(lla(1)),
        -1.0f * sind(lla(0)) * sind(lla(1)), cosd(lla(1)), -1.0f * cosd(lla(0)) * sind(lla(1)),
        cosd(lla(0)), 0, -1.0f * sind(lla(0))
    };
    return R_ET;
}

// TODO one day test
BLA::Matrix<3, 1> QuaternionUtils::ecef2nedVec(const BLA::Matrix<3, 1> &ecef_meas, const BLA::Matrix<3, 1> &launch_ecef, const BLA::Matrix<3, 3> &R_ET) {
    return ~R_ET * (ecef_meas - launch_ecef);

}

// TODO test
BLA::Matrix<3, 1> QuaternionUtils::ned2ecefVec(const BLA::Matrix<3, 1> &ned_meas, const BLA::Matrix<3, 1> &launch_ecef, const BLA::Matrix<3, 3> &R_ET) {
    return R_ET * ned_meas + launch_ecef;
}

// TODO test
BLA::Matrix<3, 1> quat2RPY(const BLA::Matrix<4, 1> &p) {
    // Not matlab, but from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // XYZ RPY
    BLA::Matrix<3, 1> RPY = {0, 0, 0};

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (p(0) * p(1) + p(2) * p(3));
    double cosr_cosp = 1 - 2 * (p(1) * p(1) + p(2) * p(2));
    RPY(0) = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (p(0) * p(2) - p(1) * p(3)));
    double cosp = std::sqrt(1 - 2 * (p(0) * p(2) - p(1) * p(3)));
    RPY(1) = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (p(0) * p(3) + p(1) * p(2));
    double cosy_cosp = 1 - 2 * (p(2) * p(2) + p(3) * p(3));
    RPY(2) = std::atan2(siny_cosp, cosy_cosp);

    return RPY;
}

// Works
BLA::Matrix<4, 1> QuaternionUtils::quatConjugate(const BLA::Matrix<4, 1> &p){
    BLA::Matrix<4, 1> quat;
    quat(0, 0) = p(0);
    quat(1, 0) = -1 * p(1);
    quat(2, 0) = -1 * p(2);
    quat(3, 0) = -1 * p(3);

    return quat;
}

// TODO test
BLA::Matrix<3, 1> QuaternionUtils::qRot(const BLA::Matrix<4, 1> &q, BLA::Matrix<3, 1> vec) {
    return QuaternionUtils::quat2DCM(q) * vec;
}

// TODO test when actually implement grav world model
BLA::Matrix<3, 1> QuaternionUtils::g_i_ecef(const BLA::Matrix<3, 3> dcm_ned2ecef) {
    // TODO one day put in the world model and take in position
    BLA::Matrix<3, 1> grav_ned = {0, 0, 9.8};
    return ~dcm_ned2ecef * grav_ned;
}

// TODO test when actually implement mag world model
BLA::Matrix<3, 1> QuaternionUtils::m_i_ecef(const BLA::Matrix<3, 3> dcm_ned2ecef) {
    // TODO one day put in the world model and take in position. School vals for now
    BLA::Matrix<3, 1> mag_ned = {19.983111, -4.8716, 46.9986145};
    return ~dcm_ned2ecef * mag_ned;
}

// TODO test when actually implement grav world model
BLA::Matrix<3, 1> QuaternionUtils::normal_i_ecef(const BLA::Matrix<3, 3> dcm_ned2ecef) {
    // TODO one day put in the world model. See g_i notes
    BLA::Matrix<3, 1> normal_ned = {0, 0, -9.8};
    return ~dcm_ned2ecef * normal_ned;
}

BLA::Matrix<3, 1> QuaternionUtils::sun_i_ecef(float t_utc) {
    BLA::Matrix<3, 1> tmp = {0, 0, 0};
}


BLA::Matrix<3, 3> QuaternionUtils::vecs2mat(const BLA::Matrix<3, 1> v1, const BLA::Matrix<3, 1> v2, const BLA::Matrix<3, 1> v3) {
    // TODO eventually replace with just using hstack
    BLA::Matrix<3, 3> ret = {v1(0, 0), v2(0), v3(0, 0),
                            v1(1, 0), v2(1, 0), v3(1, 0),
                            v1(2, 0), v2(2, 0), v3(2, 0)};

    return ret;
}


// TODO fix
BLA::Matrix<4, 1> QuaternionUtils::triad_BE(const BLA::Matrix<3, 1> v1_b, const BLA::Matrix<3, 1> v2_b, const BLA::Matrix<3, 1> v1_i, const BLA::Matrix<3, 1> v2_i) {
    // 1 is important, 2 is secondary
    BLA::Matrix<3, 1> v1_b_norm = v1_b / BLA::Norm(v1_b);
    BLA::Matrix<3, 1> v2_b_norm = v2_b / BLA::Norm(v2_b);
    BLA::Matrix<3, 1> v1_i_norm = v1_i / BLA::Norm(v1_i);
    BLA::Matrix<3, 1> v2_i_norm = v2_b / BLA::Norm(v2_i);

    // Inertial
    BLA::Matrix<3, 1> q_r = v1_i_norm;
    BLA::Matrix<3, 1> r_r = BLA::CrossProduct(v1_i_norm, v2_i_norm) / BLA::Norm(BLA::CrossProduct(v1_i_norm, v2_i_norm));
    BLA::Matrix<3, 1> s_r = BLA::CrossProduct(q_r, r_r);

    BLA::Matrix<3, 3> M_r = vecs2mat(q_r, r_r, s_r);


    // Body
    BLA::Matrix<3, 1> q_b = v1_b_norm;
    BLA::Matrix<3, 1> r_b = BLA::CrossProduct(v1_b_norm, v2_b_norm) / BLA::Norm(BLA::CrossProduct(v1_b_norm, v2_b_norm));
    BLA::Matrix<3, 1> s_b = BLA::CrossProduct(q_b, r_b);

    BLA::Matrix<3, 3> M_b = vecs2mat(q_b, r_b, s_b);

    BLA::Matrix<3, 3> R_BE = ~(M_r * ~M_b);

    return dcm2quat(R_BE);

}

BLA::Matrix<4, 1> esoq2_EB(const BLA::Matrix<3, 4> v_b, const BLA::Matrix<3, 4> v_i) {
    // Certifiable optimality, speed (microprocessor wise), determinism, and robustness to outliers
    BLA::Matrix<4, 1> q_EB = {1, 0, 0, 0};
    return q_EB;
}

// TODO test
BLA::Matrix<3,1> QuaternionUtils::lla2ecef2(BLA::Matrix<3,1> lla) {
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

// TODO test
BLA::Matrix<3, 1> QuaternionUtils::ecef2lla(BLA::Matrix<3, 1> ecef) {
    // Follows this kid idk if right, gotta test, but I'm not writing this bs from scratch
    // I think this is's impl not super precise, but not that deep (I think). LLA is stupid
    // https://github.com/kvenkman/ecef2lla/blob/master/ecef2lla.py

    float pi = 3.141592653589;
	float a = 6378137.0f;
	float a_sq = a * a;
	float e = 8.181919084261345e-2f;
	float e_sq = 6.69437999014e-3f;

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
	float r_0 = -1.0f * (p * e_sq * r) / (1.0 + q) + std::sqrt(0.5f * std::pow(a, 2) * (1.0f + (1.0f / q)) - 0.5f * p * std::pow(r, 2));

    float u = std::sqrt(std::pow(r - e_sq * r_0, 2) + std::pow(ecef(2), 2));
    float v = std::sqrt(std::pow(r - e_sq * r_0, 2) + (1.0f - e_sq) * std::pow(ecef(2), 2));
    float z_0 = std::pow(b, 2) * ecef(2) / (a * v);
    float h = u * (1.0f - std::pow(b, 2) / (a * v));
    float phi = std::atan((ecef(2) + ep_sq * z_0) / r);
    float lambd = std::atan2(ecef(1), ecef(0));

    BLA::Matrix<3, 1> lla = {phi*180.0f/pi, lambd*180.0/pi, h};
	return lla;
}

// Do this if above is inaccurate
/*
def ecef_to_lla(ecef):
    '''Convert ECEF (meters) coordinates to geodetic coordinates (degrees and meters).
    It would be good to check out the OG source and make sure this is appropriate.
    Uses those parameters right above here.

    Sourced from https://possiblywrong.wordpress.com/2014/02/14/when-approximate-is-better-than-exact/
    Originally from
    Olson, D. K., Converting Earth-Centered, Earth-Fixed Coordinates to
    Geodetic Coordinates, IEEE Transactions on Aerospace and Electronic
    Systems, 32 (1996) 473-476.

    Parameters
    ----------
    ecef : numpy.ndarray
        Position of satellite in ECEF frame.

    Returns
    -------
    list
        Latitude, longitude, and vertical distance above geode along surface normal.
    '''
    w = math.sqrt(ecef[0] * ecef[0] + ecef[1] * ecef[1])
    z = ecef[2]
    zp = abs(z)
    w2 = w * w
    r2 = z * z + w2
    r  = math.sqrt(r2)
    s2 = z * z / r2
    c2 = w2 / r2
    u = a2 / r
    v = a3 - a4 / r
    if c2 > 0.3:
        s = (zp / r) * (1 + c2 * (a1 + u + s2 * v) / r)
        lat = math.asin(s)
        ss = s * s
        c = math.sqrt(1 - ss)
    else:
        c = (w / r) * (1 - s2 * (a5 - u - c2 * v) / r)
        lat = math.acos(c)
        ss = 1 - c * c
        s = math.sqrt(ss)
    g = 1 - e2 * ss
    rg = a / math.sqrt(g)
    rf = a6 * rg
    u = w - rg * c
    v = zp - rf * s
    f = c * u + s * v
    m = c * v - s * u
    p = m / (rf / g + f)
    lat = lat + p
    if z < 0:
        lat = -lat
    return (np.degrees(lat), np.degrees(math.atan2(ecef[1], ecef[0])), f + m * p / 2)
*/

// TODO test
BLA::Matrix<4, 1> QuaternionUtils::normalizeQuaternion(BLA::Matrix<4, 1> quat) {
    BLA::Matrix<4, 1> new_quat = quat / BLA::Norm(quat);
    return new_quat;
}

float QuaternionUtils::cosd(float degs) {
    float pi = 3.141592653589;
    return cos(degs * (pi / 180.0f));
}

float QuaternionUtils::sind(float degs) {
    float pi = 3.141592653589;
    return sin(degs * (pi / 180.0f));
}

BLA::Matrix<3, 1> QuaternionUtils::combine_variances(const BLA::Matrix<3, 2> &A, const BLA::Matrix<2, 1> w) {
    BLA::Matrix<3, 1> output_variance;
    
    for (int i = 0; i < 3; i++) {
        output_variance(i) = w(0) * w(0) * A(i, 0) + w(1) * w(1) * A(i, 1);
    }
    return output_variance;
}

BLA::Matrix<3, 1> QuaternionUtils::fuse_measurements(const BLA::Matrix<3, 2> &A, const BLA::Matrix<2, 1> w) {
    BLA::Matrix<3, 1> output_val;
        
    for (int i = 0; i < 3; i++) {
        output_val(i) = w(0) * A(i, 0) + w(1) * A(i, 1);
    }
    return output_val;
}





