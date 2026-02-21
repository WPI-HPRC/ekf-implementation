#include "qmekf.h"

#include "BasicLinearAlgebra.h"

#include <Arduino.h>

#include "QuaternionUtils.h"

#include <array>

#if defined(USBCON)
#define DBG SerialUSB
#else
#define DBG Serial
#endif

using namespace QuaternionUtils;
using namespace std;

void StateEstimator::init(BLA::Matrix<3, 1> ECEF, float curr_time) {
    if (ECEF(0) != 0) {
        launch_ecef = ECEF;
    }

    x = {1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f,
            vimu_const::gyro_bias(0), vimu_const::gyro_bias(0), vimu_const::gyro_bias(0),
            vimu_const::accel_bias(0), vimu_const::accel_bias(1), vimu_const::accel_bias(2),
            icm20948_const::mag_bias(0), icm20948_const::mag_bias(1), icm20948_const::mag_bias(2),
            lps22_const::baro_bias(0)};


    // TODO idk figure this out
    P.Fill(0.0f);
    for(int i = 0 ; i < QMEKFInds::quat.Rows; i++) {
        P(QMEKFInds::quat(i, 0), QMEKFInds::quat(i, 0)) = 1e-8;
    }
	for(int i = 0 ; i < QMEKFInds::vel.Rows; i++) {
        P(QMEKFInds::vel(i, 0), QMEKFInds::vel(i, 0)) = 1e-8;
    }
	for(int i = 0 ; i < QMEKFInds::pos.Rows; i++) {
        P(QMEKFInds::pos(i, 0), QMEKFInds::pos(i, 0)) = 1e-8;
    }
    for(int i = 0 ; i < QMEKFInds::gyroBias.Rows; i++) {
        P(QMEKFInds::gyroBias(i, 0), QMEKFInds::gyroBias(i, 0)) = powf(icm20948_const::gyro_VRW, 2.0f);
    }
    for(int i = 0 ; i < QMEKFInds::accelBias.Rows; i++) {
        P(QMEKFInds::accelBias(i, 0), QMEKFInds::accelBias(i, 0)) = powf(icm20948_const::accelXY_VRW, 2.0f);
    }
    for(int i = 0 ; i < QMEKFInds::magBias.Rows; i++) {
        P(QMEKFInds::magBias(i, 0), QMEKFInds::magBias(i, 0)) = powf(0.1f, 2);
    }
    for(int i = 0 ; i < QMEKFInds::baroBias.Rows; i++) {
        P(QMEKFInds::baroBias(i, 0), QMEKFInds::baroBias(i, 0)) = powf(0.1f, 2);
    }

    launch_dcmned2ecef = dcm_ned2ecef(ecef2lla(launch_ecef));

    numLoop = 0;
    sumAccel = {0, 0, 0};
    sumMag = {0, 0, 0};

    lastTimes = {curr_time, curr_time, curr_time, curr_time, curr_time, curr_time};
}

BLA::Matrix<20, 1> StateEstimator::getState() {
    return x;
}

BLA::Matrix<20, 20> StateEstimator::getP() {
    return P;
}

BLA::Matrix<3, 1> StateEstimator::get_gyro_prev() {
    return gyro_prev;
}

BLA::Matrix<3, 1> StateEstimator::get_accel_prev() {
    return accel_prev;
}

BLA::Matrix<3, 1> StateEstimator::get_vel_prev() {
    return vel_prev;
}

BLA::Matrix<3, 1> StateEstimator::get_pos_prev() {
    return pos_prev;
}

BLA::Matrix<3, 1> StateEstimator::get_mag_prev() {
    return mag_prev;
}

BLA::Matrix<1, 1> StateEstimator::get_baro_prev() {
    return baro_prev;
}



void StateEstimator::padLoop(BLA::Matrix<3, 1> accel, BLA::Matrix<3, 1> mag, BLA::Matrix<3, 1> gps_pos) {
    sumAccel(0) = sumAccel(0) + accel(0);
    sumAccel(1) = sumAccel(1) + accel(1);
    sumAccel(2) = sumAccel(2) + accel(2);
    sumMag(0) = sumMag(0) + mag(0);
    sumMag(1) = sumMag(1) + mag(1);
    sumMag(2) = sumMag(2) + mag(2);

    if (gps_pos(0) != 0) {
        launch_ecef = gps_pos;
    }

    numLoop = numLoop + 1;
}

void StateEstimator::computeInitialOrientation() {
    if (sumMag(0, 0) == 0 && sumAccel(0, 0) == 0) {
        x(0) = 1.0f;
        x(1) = 0.0f;
        x(2) = 0.0f;
        x(3) = 0.0f;

    } else {
        BLA::Matrix<3, 1> normal_i = normal_i_ecef(launch_dcmned2ecef);
        BLA::Matrix<3, 1> m_i = m_i_ecef(launch_dcmned2ecef);
        BLA::Matrix<3, 1> normal_b = sumAccel / numLoop;
        BLA::Matrix<3, 1> m_b = sumMag / numLoop;
        BLA::Matrix initial_quat = triad_BE(normal_b, m_b, normal_i, m_i);
        
        
        x(0) = initial_quat(0);
        x(1) = initial_quat(1);
        x(2) = initial_quat(2);
        x(3) = initial_quat(3);
    }
}

BLA::Matrix<20, 1> StateEstimator::fastGyroProp(BLA::Matrix<3,1> gyro, float curr_time) {
    BLA::Matrix<4, 1> last_relevant_times = {0, 1, 3, 4};
    float dt = curr_time - vecMax(extractSub(lastTimes, last_relevant_times));
    
    float dt2 = dt/2.0f;

    BLA::Matrix<4, 1> p_q = extractSub(x, QMEKFInds::quat);
    // TODO combine gyro through vimu

    BLA::Matrix<3, 1> unbiased_gyro = gyro - extractSub(x, QMEKFInds::gyroBias);
    // OLD METHOD BROKEN BLA::Matrix<4, 1> new_q = quatMultiply(extractSub(x, QMEKFInds::quat), rotVec2dQuat(unbiased_gyro * dt));

    BLA::Matrix<4, 1> new_q = {
        (p_q(0) - dt2*(unbiased_gyro(0))*p_q(1) - dt2*(unbiased_gyro(1))*p_q(2) - dt2*(unbiased_gyro(2))*p_q(3)),
        (p_q(1) + dt2*(unbiased_gyro(0))*p_q(0) - dt2*(unbiased_gyro(1))*p_q(3) + dt2*(unbiased_gyro(2))*p_q(2)),
        (p_q(2) + dt2*(unbiased_gyro(0))*p_q(3) + dt2*(unbiased_gyro(1))*p_q(0) - dt2*(unbiased_gyro(2))*p_q(1)),
        (p_q(3) - dt2*(unbiased_gyro(0))*p_q(2) + dt2*(unbiased_gyro(1))*p_q(1) + dt2*(unbiased_gyro(2))*p_q(0))
    };
    new_q = new_q / BLA::Norm(new_q);
    x = setSub(x, QMEKFInds::quat, new_q);
    gyro_prev = unbiased_gyro;
    lastTimes(0) = curr_time;
    return x;
}

BLA::Matrix<20, 1> StateEstimator::fastAccelProp(BLA::Matrix<3, 1> accel, float curr_time) {
    BLA::Matrix<3, 1> last_relevant_times = {1, 4, 5};
    float dt = curr_time - vecMax(extractSub(lastTimes, last_relevant_times));

    // TODO combine accel through vimu

    BLA::Matrix<3, 1> unbiased_accel = accel - extractSub(x, QMEKFInds::accelBias);
    //DBG.print("unbiased accel: "); DBG.println(unbiased_accel);
    BLA::Matrix<3, 3> q_conj_dcm = quat2DCM(quatConjugate(extractSub(x, QMEKFInds::quat)));
    BLA::Matrix<3, 1> v = {((q_conj_dcm * unbiased_accel + q_conj_dcm * accel_prev) / 2.0f) + g_i_ecef(launch_dcmned2ecef) * dt + vel_prev};
    DBG.print("v: "); DBG.println(v);
    BLA::Matrix<3, 1> p = {((v + vel_prev) / 2.0f) * dt + pos_prev};
    x = setSub(x, QMEKFInds::vel, v);
    x = setSub(x, QMEKFInds::pos, p);

    accel_prev = unbiased_accel;
    vel_prev = v;
    pos_prev = p;
    lastTimes(1) = curr_time;
    return x;
}

BLA::Matrix<20, 20> StateEstimator::ekfPredict(float curr_time) {
    // TODO finish this. too lazy
    BLA::Matrix<4, 1> att_last_relevant_times = {0, 1, 3, 4};
    BLA::Matrix<4, 1> pv_last_relevant_times = {1, 4, 5};
    float att_dt = curr_time - vecMax(extractSub(lastTimes, att_last_relevant_times));
    float pv_dt = curr_time - vecMax(extractSub(lastTimes, pv_last_relevant_times));


    BLA::Matrix<3,3> gyroSkew = skewSymmetric(gyro_prev);
    BLA::Matrix<3,3> accelSkew = skewSymmetric(accel_prev);

    BLA::Matrix<3, 3> q_conj_dcm = quat2DCM(quatConjugate(extractSub(x, QMEKFInds::quat)));

    BLA::Matrix<20, 20> F;
    F.Fill(0);

    //Row 1 - 3
    F.Submatrix<3, 3>(0, QMEKFInds::q_w) = -1.0f * gyroSkew; 
    F.Submatrix<3, 3>(0, QMEKFInds::gb_x - 1) = -1.0f * I_3;

    //Row 4 - 6
    F.Submatrix<3, 3>(QMEKFInds::v_x - 1, QMEKFInds::q_w) = -1.0f * q_conj_dcm * accelSkew;
    F.Submatrix<3, 3>(QMEKFInds::v_x - 1, QMEKFInds::ab_x - 1) = -1.0f * q_conj_dcm;

    //Row 7 - 9
    F.Submatrix<3, 3>(QMEKFInds::p_x - 1, QMEKFInds::v_x - 1) = I_3;
    
    BLA::Matrix<20, 20> phi;
    phi.Fill(0);

    phi = I_20 + (F * pv_dt) + (0.5f * F * F * float(pow(pv_dt, 2))); // I was hoping to split up the dt. Not that deep tho

    BLA::Matrix<20, 20> phi_t = ~phi;

    // Process noise
    BLA::Matrix<20, 20> Q_d;
    Q_d.Fill(0);


    //Sorry random numbers theyre in the output.txt file from the allan variance tests
    BLA::Matrix<3, 3> gyro_var_diag;
    gyro_var_diag.Fill(0);
    gyro_var_diag(0, 0) = 14.00553 / 1000.0f * (PI / 180.0f) / sqrtf(pv_dt);
    gyro_var_diag(1, 1) = 1124.893 / 1000.0f * (PI / 180.0f) / sqrtf(pv_dt);
    gyro_var_diag(2, 2) = 6.534220 / 1000.0f * (PI / 180.0f) / sqrtf(pv_dt);

    BLA::Matrix<3, 3> gyro_bias_var_diag;
    gyro_bias_var_diag.Fill(0);
    gyro_bias_var_diag(0, 0) = 0.1152665 / 1000.0f * (PI / 180.0f);
    gyro_bias_var_diag(1, 1) = 0.1621635 / 1000.0f * (PI / 180.0f);
    gyro_bias_var_diag(2, 2) = 0.09894897 / 1000.0f * (PI / 180.0f);

    BLA::Matrix<3, 3> accel_var_diag;
    accel_var_diag.Fill(0);
    accel_var_diag(0, 0) = 103.2008 * 0.00980665f / sqrtf(pv_dt);
    accel_var_diag(1, 1) = 0.1732320 * 0.00980665f / sqrtf(pv_dt);
    accel_var_diag(2, 2) = 0.2295548 * 0.00980665f / sqrtf(pv_dt);

    BLA::Matrix<3, 3> accel_bias_var_diag;
    accel_bias_var_diag.Fill(0);
    accel_bias_var_diag(0, 0) = 0.01474176 * 0.00980665f;
    accel_bias_var_diag(1, 1) = 0.03503381 * 0.00980665f;
    accel_bias_var_diag(2, 2) = 0.0043195624 * 0.00980665f;

    BLA::Matrix<3, 3> mag_bias_var_diag;
    mag_bias_var_diag.Fill(0);
    mag_bias_var_diag(0, 0) = 0.008301463f;
    mag_bias_var_diag(1, 1) = 0.003226824f;
    mag_bias_var_diag(2, 2) = 0.01745155f;

    BLA::Matrix<2, 2> baro_bias_var_diag;
    baro_bias_var_diag.Fill(0);
    baro_bias_var_diag(0, 0) = 0.01243823;
    baro_bias_var_diag(1, 1) = 0.008426440f;

    Q_d.Submatrix<3, 3>(QMEKFInds::q_w, QMEKFInds::q_w) = (gyro_var_diag * att_dt) + (gyro_bias_var_diag * float((pow(att_dt, 3) / 10)));
    Q_d.Submatrix<3, 3>(QMEKFInds::q_w, 9) = -1.0f * gyro_bias_var_diag * float((pow(att_dt, 2) / 2));

    Q_d.Submatrix<3 ,3>(3, 3) = accel_var_diag * pv_dt + accel_bias_var_diag * float((pow(pv_dt, 3) / 3));
    Q_d.Submatrix<3, 3>(3, 6) = accel_bias_var_diag * float((pow(pv_dt ,4) / 8.0)) + accel_var_diag * float((pow(pv_dt, 2) / 2.0));
    Q_d.Submatrix<3, 3>(3, 10) = -1.0f * accel_bias_var_diag * float((pow(pv_dt, 2) / 2.0));

    Q_d.Submatrix<3, 3>(6, 3) = accel_var_diag * float((pow(pv_dt, 2) / 2)) + accel_bias_var_diag * float((pow(pv_dt, 4) / 8.0));
    Q_d.Submatrix<3, 3>(6, 6) = accel_var_diag * float((pow(pv_dt, 3) / 3.0)) + accel_bias_var_diag * float((pow(pv_dt, 5) / 20.0));
    Q_d.Submatrix<3, 3>(6, 10) = -1.0f * accel_bias_var_diag * float((pow(pv_dt, 3) / 6.0));

    Q_d.Submatrix<3, 3>(9, 0) = -1.0f * gyro_bias_var_diag * float((pow(att_dt, 2) / 2.0));
    Q_d.Submatrix<3, 3>(9, 9) = gyro_bias_var_diag * float((pow(att_dt, 2) / 2.0));

    Q_d.Submatrix<3, 3>(12, 3) = -1.0f * accel_bias_var_diag * float((pow(pv_dt, 2) / 2.0));
    Q_d.Submatrix<3, 3>(12, 6) = -1.0f * accel_bias_var_diag * float((pow(pv_dt, 2) / 2.0));
    Q_d.Submatrix<3, 3>(12, 12) = accel_bias_var_diag * pv_dt;

    Q_d.Submatrix<3, 3>(15, 15) = mag_bias_var_diag * pv_dt;
    Q_d.Submatrix<2, 2>(18, 18) = baro_bias_var_diag * pv_dt;

    BLA::Matrix<20, 20> P;
    
    P = phi * P * phi_t + Q_d;

    return P;
    
}

BLA::Matrix<20, 1> StateEstimator::runAccelUpdate(BLA::Matrix<3, 1> a_b, float curr_time) {
    BLA::Matrix<3, 1> unbiased_accel = a_b - extractSub(x, QMEKFInds::accelBias);

    BLA::Matrix<4,1> q = extractSub(x, QMEKFInds::quat);

    BLA::Matrix<3, 1> h_accel = quat2DCM(q) * normal_i_ecef(launch_dcmned2ecef);

    BLA::Matrix<3, 20> H_accel;
    H_accel.Fill(0);
    H_accel.Submatrix<3, 3>(0, 0) = skewSymmetric(h_accel);
    H_accel.Submatrix<3, 3>(0, QMEKFInds::ab_x - 1) = I_3;

    BLA::Matrix<3, 3> R = toDiag(icm20948_const::accel_var);

    lastTimes(2) = curr_time;
    return ekfCalcErrorInject(unbiased_accel, H_accel, h_accel, R);
}

BLA::Matrix<20, 1> StateEstimator::runMagUpdate(BLA::Matrix<3, 1> m_b, float curr_time) {
    BLA::Matrix<3, 1> unbiased_accel = m_b - extractSub(x, QMEKFInds::magBias);

    BLA::Matrix<4,1> q = extractSub(x, QMEKFInds::quat);

    BLA::Matrix<3, 1> h_mag = quat2DCM(q) * m_i_ecef(launch_dcmned2ecef);

    BLA::Matrix<3, 20> H_mag;
    H_mag.Fill(0);
    H_mag.Submatrix<3, 3>(0, 0) = skewSymmetric(h_mag);
    H_mag.Submatrix<3, 3>(0, QMEKFInds::mb_x - 1) = I_3;

    BLA::Matrix<3, 3> R = toDiag(icm20948_const::mag_var);

    lastTimes(3) = curr_time;
    return ekfCalcErrorInject(unbiased_accel, H_mag, h_mag, R);
}

BLA::Matrix<20, 1> StateEstimator::runGPSUpdate(BLA::Matrix<3, 1> gpsPos, BLA::Matrix<3, 1> gpsVel, bool velOrientation, float curr_time) {
    // TODO something is wrong with gps vel
    lastTimes(4) = curr_time;
    if (velOrientation) {
        BLA::Matrix<9, 1> combined_sens = vstack(vstack(gpsVel, gpsPos), gpsVel);
        BLA::Matrix<3, 1> v_b = {BLA::Norm(gpsVel), 0, 0};
        BLA::Matrix<4, 1> q = extractSub(x, QMEKFInds::quat);
        BLA::Matrix<3, 1> h_vi = quat2DCM(quatConjugate(q)) * v_b;
        BLA::Matrix<9, 1> h_gps = vstack(extractSub(x, vstack(QMEKFInds::vel, QMEKFInds::pos)), extractSub(x, QMEKFInds::vel));
        
        BLA::Matrix<9, 20> H_gps;
        H_gps.Fill(0);
        H_gps.Submatrix<3, 3>(0, QMEKFInds::v_x - 1) = I_3;
        H_gps.Submatrix<3, 3>(3, QMEKFInds::p_x - 1) = I_3;
        H_gps.Submatrix<3, 3>(6, 0) = -1.0f * quat2DCM(quatConjugate(q)) * skewSymmetric(v_b);

        BLA::Matrix<9, 9> R = toDiag(vstack(vstack(Max10S_const::gpsVel_var, Max10S_const::gpsPos_var), Max10S_const::gpsVel_var));

        return ekfCalcErrorInject(combined_sens, H_gps, h_gps, R);
    } else {
        BLA::Matrix<6, 1> combined_sens = vstack(gpsVel, gpsPos);
        BLA::Matrix<6, 20> H_gps;
        H_gps.Fill(0);
        H_gps.Submatrix<3, 3>(0, QMEKFInds::v_x - 1) = I_3;
        H_gps.Submatrix<3, 3>(3, QMEKFInds::p_x - 1) = I_3;

        BLA::Matrix<6, 1> h_gps = extractSub(x, vstack(QMEKFInds::vel, QMEKFInds::pos));

        BLA::Matrix<6, 6> R = toDiag(vstack(Max10S_const::gpsVel_var, Max10S_const::gpsPos_var));
        return ekfCalcErrorInject(combined_sens, H_gps, h_gps, R);
    }
    
}

BLA::Matrix<20, 1> StateEstimator::runBaroUpdate(BLA::Matrix<1, 1> baro, float curr_time) {

    BLA::Matrix<3, 1> lla = ecef2lla(extractSub(x, QMEKFInds::pos));

    BLA::Matrix<1, 1> h_baro = {std::pow(lps22_const::P0 * (1.0f + (lps22_const::L * lla(2))) / lps22_const::T0, -1.0f * lps22_const::g_e * lps22_const::M / (lps22_const::R * lps22_const::L))};

    float dP_dh = (-1.0f * lps22_const::g_e * h_baro(0, 0) * lps22_const::M) / (lps22_const::R * (lps22_const::T0 + lps22_const::L * lla(2)));
    BLA::Matrix<3, 1> dh_decef = {cosd(lla(0)) * cosd(lla(1)), cosd(lla(0)) * sind(lla(1)), sind(lla(0))};
    BLA::Matrix<3, 1> dP_decef = dP_dh * dh_decef;

    BLA::Matrix<1, 20> H_baro;
    H_baro.Fill(0);
    H_baro.Submatrix<1, 3>(0, QMEKFInds::p_x - 1) = ~dP_decef;
    H_baro(0, 18) = 1.0f;
    H_baro(0, 19) = 1.0f;

    BLA::Matrix<1, 1> R = lps22_const::baro_var;

    lastTimes(5) = curr_time;
    return ekfCalcErrorInject(baro, H_baro, h_baro, R);
}

void StateEstimator::setTemp(float new_temp) {
    curr_temp = new_temp;
}

float StateEstimator::getTemp() {
    return curr_temp;
}

template<int rows>
BLA::Matrix<20, 1> StateEstimator::ekfCalcErrorInject(BLA::Matrix<rows, 1> &sens_reading, BLA::Matrix<rows, 20> H, BLA::Matrix<rows, 1> h, BLA::Matrix<rows, rows> R) {
    BLA::Matrix<rows, 1> residual = sens_reading - h;
    
    BLA::Matrix<rows, rows> S;
    BLA::Matrix<20, rows> K;
    BLA::Matrix<20, rows> H_t = ~H;
    
    S = H * P * H_t + R;
    K = (P * H_t) * BLA::Inverse(S);
    BLA::Matrix<20, 1> postErrorState = K * residual;
    
    // Inject error angles into quat
    BLA::Matrix<3, 1> alpha = extractSub(postErrorState, QMEKFInds::smallAngle);

    BLA::Matrix old_q = extractSub(x, QMEKFInds::quat);
    BLA::Matrix<4, 1> q = quatMultiply(old_q, smallAnglerotVec2dQuat(alpha));
    
    x(0) = q(0);
    x(1) = q(1);
    x(2) = q(2);
    x(3) = q(3);
    x(4) += postErrorState(3);
    x(5) += postErrorState(4);
    x(6) += postErrorState(5);
    x(7) += postErrorState(6);
    x(8) += postErrorState(7);
    x(9) += postErrorState(8);
    x(10) += postErrorState(9);
    x(11) += postErrorState(10);
    x(12) += postErrorState(11);
    x(13) += postErrorState(12);
    x(14) += postErrorState(13);
    x(15) += postErrorState(14);
    x(16) += postErrorState(15);
    x(17) += postErrorState(16);
    x(18) += postErrorState(17);
    x(19) += postErrorState(18);
    x(20) += postErrorState(19);
    
    return x;
}

BLA::Matrix<4, 1> StateEstimator::getNEDOrientation(BLA::Matrix<3, 3> &dcm_ned2ecef) {
    // TODO inefficient, use quat mult
    return dcm2quat(~dcm_ned2ecef * quat2DCM(extractSub(x, QMEKFInds::quat)));
}

BLA::Matrix<3, 1> StateEstimator::getNEDPositionBody(BLA::Matrix<3, 3> &dcm_ned2ecef, BLA::Matrix<3, 1> launch_ecef) {
    // Returns the distance from 
    return ecef2nedVec(extractSub(x, QMEKFInds::pos) - qRot(extractSub(x, QMEKFInds::quat), mars_const::bodyToVIMUDis), launch_ecef, dcm_ned2ecef);
}

BLA::Matrix<3, 1> StateEstimator::getBodyAngularVel() {
    return gyro_prev;
}

BLA::Matrix<3, 1> StateEstimator::getVIMUAccel() {
    return accel_prev;
}

float StateEstimator::getGs() {
    return BLA::Norm(getVIMUAccel());
}

// TODO for all of these
float getLastAttProp() {
    // TODO
    return 0.0f;
}

float getLastAttUpdate() {
    // TODO
    return 0.0f;
}

float getLastPVProp() {
    // TODO
    return 0.0f;
}

float getLastPVUpdate() {
    // TODO
    return 0.0f;
}

// GPS is less accurate in D direction, so account for that here. Vel then pos
BLA::Matrix<6, 1> getGPSVar(BLA::Matrix<3, 3> dcm_ned2ecef) {
    BLA::Matrix<6, 1> tmp = {0, 1, 2};
    return tmp;
    // return dcm_ned2ecef * gps_var;

}