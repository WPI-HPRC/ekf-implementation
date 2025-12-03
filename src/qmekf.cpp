#include "qmekf.h"

#include "BasicLinearAlgebra.h"

#include <Arduino.h>

#include "QuaternionUtils.h"

void StateEstimator::init(BLA::Matrix<3, 1> ECEF, float curr_time) {
    if (ECEF(0) != 0) {
        launch_ecef = ECEF;
    }

    x = {0, 0, 0, 0,
            0, 0, 0,
            launch_ecef(0), launch_ecef(1), launch_ecef(2),
            vimu_const::gyro_bias(0), vimu_const::gyro_bias(0), vimu_const::gyro_bias(0),
            vimu_const::accel_bias(0), vimu_const::accel_bias(1), vimu_const::accel_bias(2),
            icm20948_const::mag_bias(0), icm20948_const::mag_bias(1), icm20948_const::mag_bias(2),
            lps22_const::baro_bias(0)};


    // TODO idk figure this out
    P.Fill(0.0f);
    for(uint8_t idx : QMEKFInds::quat) {
        P(idx, idx) = 1e-8;
    }
	for (uint8_t idx : QMEKFInds::vel) {
		P(idx, idx) = 1e-8;
	}
	for (uint8_t idx : QMEKFInds::pos) {
		P(idx, idx) = 1e-8;
	}
    for(uint8_t idx : QMEKFInds::gyroBias) {
        P(idx, idx) = powf(icm20948_const::gyro_VRW, 2.0f);
    }
    for(uint8_t idx: QMEKFInds::accelBias) {
        P(idx, idx) = powf(icm20948_const::accelXY_VRW, 2.0f);
    }
    for(uint8_t idx : QMEKFInds::magBias) {
        P(idx, idx) = powf(0.1f, 2);
    }
	for (uint8_t idx : QMEKFInds::baroBias) {
		P(idx, idx) = powf(0.1f, 2);
	}

    launch_dcmned2ecef = QuaternionUtils::dcm_ned2ecef(QuaternionUtils::ecef2lla(launch_ecef));

    numLoop = 0;
    sumAccel = {0, 0, 0};
    sumMag = {0, 0, 0};

    lastTimes = {curr_time, curr_time, curr_time, curr_time, curr_time, curr_time};
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
    BLA::Matrix<3, 1> normal_i = QuaternionUtils::normal_i_ecef(launch_dcmned2ecef);
    BLA::Matrix<3, 1> m_i = QuaternionUtils::m_i_ecef(launch_dcmned2ecef);
    BLA::Matrix<3, 1> normal_b = sumAccel / numLoop;
    BLA::Matrix<3, 1> m_b = sumMag / numLoop;
    BLA::Matrix initial_quat = QuaternionUtils::triad_BE(normal_b, m_b, normal_i, m_i);
    
    
    x(0) = initial_quat(0);
    x(1) = initial_quat(1);
    x(2) = initial_quat(2);
    x(3) = initial_quat(3);
}

BLA::Matrix<20, 1> StateEstimator::fastGyroProp(BLA::Matrix<3,1> gyro, float curr_time) {
    // float dt = curr_time - QuaternionUtils::vecMax(QuaternionUtils::extractSub(lastTimes, {0, 1, 3, 4}));;
    BLA::Matrix<3, 1> unbiased_gyro = gyro;


    gyro_prev = unbiased_gyro;

    return x;
}


// BLA::Matrix<20,1> StateEstimator::onLoop(int state) {
//     // Read data from sensors and convert values
    
//     float dt;

//     float gyrX = IMUData->gyrX;
//     float gyrY = IMUData->gyrY;
//     float gyrZ = IMUData->gyrZ;

//     float aclX = IMUData->accelX;
//     float aclY = IMUData->accelY;
//     float aclZ = IMUData->accelZ;

//     float magX = IMUData->magX;
//     float magY = IMUData->magY;
//     float magZ = IMUData->magZ;
    
	
// 	// TODO get in the GPS data and baro data from somewhere
//     BLA::Matrix<3,1> gyro = {gyrX, gyrY, gyrZ};   // [rad/s]
//     BLA::Matrix<3,1> accel = {aclX, aclY, aclZ}; // [m/s^s]
//     BLA::Matrix<3,1> mag = {magX, magY, magZ}; // [uT]
//     BLA::Matrix<3,1> gpsData = {gpsData(0), gpsData(1), gpsData(2)}; 
// 	// BLA::Matrix<1,1> baro = {baroZ};
	
// 	// Remove biases from each measurement
// 	BLA::Matrix<3,1> unbiased_gyro = {gyro(0) - x(QMEKFInds::gb_x), gyro(1) - x(QMEKFInds::gb_y), gyro(2) - x(QMEKFInds::gb_z)};
// 	BLA::Matrix<3,1> unbiased_accel = {accel(0) - x(QMEKFInds::ab_x), accel(1) - x(QMEKFInds::ab_y), accel(2) - x(QMEKFInds::ab_z)};
// 	BLA::Matrix<3,1> unbiased_mag = {mag(0) - x(QMEKFInds::mb_x), mag(1) - x(QMEKFInds::mb_y), mag(2) - x(QMEKFInds::mb_z)};
// 	// BLA::Matrix<1,1> unbiased_baro = {baro(0) - x(20)};
	
	
// 	float time = millis();
// 	bool run_priori = time - lastTimes(0) >= frequencies(0);
// 	bool run_accel_update = time - lastTimes(1) >= frequencies(1);
// 	bool run_mag_update = time - lastTimes(2) >= frequencies(2);
// 	bool run_gps_update = time - lastTimes(3) >= frequencies(3);
// 	// bool run_baro_update = time - lastTimeBaro(4) >= frequencies(4);

    
// 	if(run_priori) {
// 		// TODO eventually implement RK4 here, but I don't understand it yet
// 		float lastAttUpdate = max(max(lastTimes(0), lastTimes(1)), lastTimes(2)); // Maximum of the lastTimes(0, 1, 2)
// 		float lastPVUpdate = max(max(lastTimes(0), lastTimes(3)), lastTimes(4)); // Maximum of the lastTimes(0, 3, 4)
// 		float dt_att = millis() - lastAttUpdate;
// 		float dt_pv = millis() - lastPVUpdate;
		
// 		x = fastIMUProp(gyro, accel, dt_att, dt_pv);
		
		
// 		lastTimes(0) = millis();
// 	}
	
// 	if (run_accel_update || run_mag_update || run_gps_update) {
// 		dt = millis() - max(max(lastTimes(0), lastTimes(1)), max(lastTimes(2), lastTimes(3)));;
//         P = predictionFunction(P, gyro, accel, dt);
// 	}
	
// 	if (run_accel_update) {
// 		runAccelUpdate(x, accel);
// 		lastTimes(1) = millis();
// 	}
	
// 	if (run_mag_update) {
// 		runMagUpdate(x, mag);
// 		lastTimes(2) = millis();
// 	}
	
// 	if (run_gps_update) {
//         BLA::Matrix<3,1> gps_ecef = QuaternionUtils::lla2ecef(gpsData);
// 		runGPSUpdate(x, gps_ecef);
// 		lastTimes(3) = millis();
// 	}
	
// 	// if (run_baro_update) {
// 	// 	run_baro_update();
// 	// 	lastTimes(4) = millis();
// 	// }

//     //Update sensor readings
//     gyro_prev = unbiased_gyro;
//     accel_prev = unbiased_accel;
//     mag_prev = unbiased_mag;
//     gps_prev = gpsData;
//     // baro_prev = unbiased_baro;

//     return x;
// }

// BLA::Matrix<20,1> StateEstimator::fastIMUProp(BLA::Matrix<3,1> gyro, BLA::Matrix<3, 1> accel, float att_dt, float pv_dt) {
    
//     BLA::Matrix<3, 1> v;
//     BLA::Matrix<3, 1> p;

// 	// TODO change to from world model when go to ECEF
//     BLA::Matrix<3,1> gyro_int = {gyro(0)*att_dt, gyro(1)*att_dt, gyro(2)*att_dt};
//     float rotVecNorm = BLA::Norm(gyro_int);
//     BLA::Matrix<3,1> axis = {(gyro_int(0) / rotVecNorm), (gyro_int(1) / rotVecNorm), (gyro_int(2) / rotVecNorm)};
//     BLA::Matrix<4,1> dq = 
//     {
//         cos(rotVecNorm/2.0f),
//         axis(0) * sinf(rotVecNorm/2.0f),
//         axis(1) * sinf(rotVecNorm/2.0f),
//         axis(2) * sinf(rotVecNorm/2.0f),
//     };
//     BLA::Matrix<4,1> q = 
//     {
//         x(QMEKFInds::q_w),
//         x(QMEKFInds::q_x),
//         x(QMEKFInds::q_y),
//         x(QMEKFInds::q_z),
//     };
//     q = QuaternionUtils::quatMultiply(q, dq);
//     BLA::Matrix<4,1> qNorm = {q(0) / BLA::Norm(q), q(1) / BLA::Norm(q), q(2) / BLA::Norm(q), q(3) / BLA::Norm(q)};
	
	
	
// 	BLA::Matrix<3,1> v_dot = QuaternionUtils::quatToRot(q) * accel + g_i;
// 	BLA::Matrix<3,1> old_v = {
// 		x(QMEKFInds::v_x), x(QMEKFInds::v_y), x(QMEKFInds::v_z)
// 	};
// 	BLA::Matrix<3,1> old_p = {
// 		x(QMEKFInds::p_x), x(QMEKFInds::p_y), x(QMEKFInds::p_z)
// 	};

// 	v = old_v + v_dot * pv_dt;
// 	p = old_p + v * pv_dt;
	
// 	x(QMEKFInds::v_x) = v(0);
// 	x(QMEKFInds::v_y) = v(1);
// 	x(QMEKFInds::v_z) = v(2);
// 	x(QMEKFInds::p_x) = p(0);
// 	x(QMEKFInds::p_y) = p(1);
// 	x(QMEKFInds::p_z) = p(2);

//     return x;

// }

// BLA::Matrix<19, 19> StateEstimator::predictionFunction(BLA::Matrix<19, 19> P_, BLA::Matrix<3, 1> accelVec, BLA::Matrix<3, 1> gyroVec, float dt) {
//     BLA::Matrix<3,3> gyroSkew = QuaternionUtils::skewSymmetric(gyroVec);
//     BLA::Matrix<3,3> accelSkew = QuaternionUtils::skewSymmetric(accelVec);

//     BLA::Matrix<4, 1> q =
//     {
//         x(QMEKFInds::q_w),
//         x(QMEKFInds::q_x),
//         x(QMEKFInds::q_y),
//         x(QMEKFInds::q_z)
//     };
    
//     BLA::Matrix<3, 3> rotMatrix = QuaternionUtils::quatToRot(q);

//     BLA::Matrix<19, 19> F;
//     F.Fill(0);

//     //Row 1 - 3
//     F.Submatrix<3, 3>(0, QMEKFInds::q_w) = -1.0f * gyroSkew; 
//     F.Submatrix<3, 3>(0, QMEKFInds::gb_x) = -1.0f * I_3;

//     //Row 4 - 6
//     F.Submatrix<3, 3>(QMEKFInds::v_x - 1, QMEKFInds::q_w) = -1.0f * rotMatrix * accelSkew;
//     F.Submatrix<3, 3>(QMEKFInds::v_x - 1, QMEKFInds::ab_x) = -1.0f * rotMatrix;

//     //Row 7 - 9
//     F.Submatrix<3, 3>(QMEKFInds::p_x -1, 3) = I_3;
    
//     BLA::Matrix<19, 19> phi;
//     phi.Fill(0);

//     phi = I_19 + (F * dt) + (0.5f * F * F * float(pow(dt, 2)));

//     BLA::Matrix<19, 19> phi_t = ~phi;

//     BLA::Matrix<19, 19> Q_d;
//     Q_d.Fill(0);

//     BLA::Matrix<3, 3> gyro_var_diag;
//     gyro_var_diag.Fill(0);
//     gyro_var_diag(0, 0) = QMEKFInds::gyro_var;
//     gyro_var_diag(1, 1) = QMEKFInds::gyro_var;
//     gyro_var_diag(2, 2) = QMEKFInds::gyro_var;

//     BLA::Matrix<3, 3> gyro_bias_var_diag;
//     gyro_bias_var_diag.Fill(0);
//     gyro_bias_var_diag(0, 0) = QMEKFInds::gyro_bias_var;
//     gyro_bias_var_diag(1, 1) = QMEKFInds::gyro_bias_var;
//     gyro_bias_var_diag(2, 2) = QMEKFInds::gyro_bias_var;

//     BLA::Matrix<3, 3> accel_bias_var_diag;
//     accel_bias_var_diag.Fill(0);
//     accel_bias_var_diag(0, 0) = QMEKFInds::accel_bias_var;
//     accel_bias_var_diag(1, 1) = QMEKFInds::accel_bias_var;
//     accel_bias_var_diag(2, 2) = QMEKFInds::accel_bias_var;

//     Q_d.Submatrix<3, 3>(QMEKFInds::q_w, QMEKFInds::q_w) = (gyro_var_diag * dt) + (gyro_bias_var_diag * float((pow(dt, 3) / 10)));
//     Q_d.Submatrix<3, 3>(QMEKFInds::q_w, 9) = -1.0f * gyro_bias_var_diag * float((pow(dt, 2) / 2));

//     BLA::Matrix<3, 3> R_grav_diag; 
//     R_grav_diag.Fill(0);
//     R_grav_diag(0, 0) = QMEKFInds::R_grav;
//     R_grav_diag(1, 1) = QMEKFInds::R_grav;
//     R_grav_diag(2, 2) = QMEKFInds::R_grav;

//     Q_d.Submatrix<3 ,3>(3, 3) = R_grav_diag * dt + accel_bias_var_diag * float((pow(dt, 3) / 3));
//     Q_d.Submatrix<3, 3>(3, 6) = accel_bias_var_diag * float((pow(dt ,4) / 8.0)) + R_grav_diag * float((pow(dt, 2) / 2.0));
//     Q_d.Submatrix<3, 3>(3, 10) = -1.0f * accel_bias_var_diag * float((pow(dt, 2) / 2.0));

//     Q_d.Submatrix<3, 3>(6, 3) = R_grav_diag * float((pow(dt, 2) / 2)) + accel_bias_var_diag * float((pow(dt, 4) / 8.0));
//     Q_d.Submatrix<3, 3>(6, 6) = R_grav_diag * float((pow(dt, 3) / 3.0)) + accel_bias_var_diag * float((pow(dt, 5) / 20.0));
//     Q_d.Submatrix<3, 3>(6, 10) = -1.0f * accel_bias_var_diag * float((pow(dt, 3) / 6.0));

//     Q_d.Submatrix<3, 3>(9, 0) = -1.0f * gyro_bias_var_diag * float((pow(dt, 2) / 2.0));
//     Q_d.Submatrix<3, 3>(9, 9) = gyro_bias_var_diag * float((pow(dt, 2) / 2.0));

//     Q_d.Submatrix<3, 3>(12, 3) = -1.0f * accel_bias_var_diag * float((pow(dt, 2) / 2.0));
//     Q_d.Submatrix<3, 3>(12, 6) = -1.0f * accel_bias_var_diag * float((pow(dt, 2) / 2.0));
//     Q_d.Submatrix<3, 3>(12, 12) = accel_bias_var_diag * dt;

//     Q_d(15, 15) = QMEKFInds::mag_bias_var * dt;

//     Q_d(18, 18) = QMEKFInds::baro_bias_var * dt;

//     BLA::Matrix<19, 19> P;
    
//     P = phi * P_ * phi_t + Q_d;

//     return P;

// }

// BLA::Matrix<20, 1> StateEstimator::runAccelUpdate(BLA::Matrix<20, 1> &x, BLA::Matrix<3,1> accel_meas)
// {
//     BLA::Matrix<4,1> q = 
//     {
//         x(QMEKFInds::q_w),
//         x(QMEKFInds::q_x),
//         x(QMEKFInds::q_y),
//         x(QMEKFInds::q_z),
//     };

//     BLA::Matrix<3, 19> H_accel;
//     H_accel.Fill(0);
//     H_accel.Submatrix<3, 3>(0, 0) = QuaternionUtils::skewSymmetric(QuaternionUtils::quat2DCM(q) * (-1.0f * normal_i));
//     H_accel.Submatrix<3, 3>(0, QMEKFInds::ab_x - 1) = -1.0f * QuaternionUtils::quat2DCM(q);

//     BLA::Matrix<3, 1> h_accel;

//     h_accel = QuaternionUtils::quat2DCM(q) * normal_i;

//     EKFCalcErrorInject(x, P, accel_meas, H_accel, h_accel, R_accel);
    
// }

// BLA::Matrix<20, 1> StateEstimator::runMagUpdate(BLA::Matrix<20, 1> &x, BLA::Matrix<3, 1> mag_meas) {
//     // TODO input igrm model somehow figure out
//     BLA::Matrix<4, 1> q =
//     {
//         x(QMEKFInds::q_w),
//         x(QMEKFInds::q_x),
//         x(QMEKFInds::q_y),
//         x(QMEKFInds::q_z)
//     };

//     BLA::Matrix<3, 19> H_mag;
//     H_mag.Fill(0);
//     H_mag.Submatrix<3, 3>(0, 0) =  QuaternionUtils::skewSymmetric(QuaternionUtils::quat2DCM(q) * m_i);
//     H_mag.Submatrix<3, 3>(0, QMEKFInds::gb_x) = I_3;

//     BLA::Matrix<3, 1> h_mag;

//     h_mag = QuaternionUtils::quat2DCM(q) * m_i;

//     EKFCalcErrorInject(x, P, mag_meas, H_mag, h_mag, R_mag);
    
// }

// BLA::Matrix<20, 1> StateEstimator::runGPSUpdate(BLA::Matrix<20, 1> &x, BLA::Matrix<3, 1> gps_meas_ecef) {
//     BLA::Matrix<3, 1> pos_ned = QuaternionUtils::ecef2ned(gps_meas_ecef, launch_ecef, R_ET);

//     BLA::Matrix<3, 19> H_gps;
//     H_gps.Fill(0);
//     H_gps.Submatrix<3, 3>(0, QMEKFInds::p_x) = I_3;

//     BLA::Matrix<3, 1> h_gps = {
//         x(QMEKFInds::p_x),
//         x(QMEKFInds::p_y),
//         x(QMEKFInds::p_z),
//     };

//     EKFCalcErrorInject(x, P, pos_ned, H_gps, h_gps, R_gps);
    
// }

// BLA::Matrix<20, 1> StateEstimator::EKFCalcErrorInject(BLA::Matrix<20, 1> &oldState, BLA::Matrix<19, 19> &oldP, BLA::Matrix<3, 1> &sens_reading, BLA::Matrix<3, 19> H, BLA::Matrix<3, 1> h, BLA::Matrix<3, 3> R) {
//     BLA::Matrix<3, 1> residual;
//     residual = sens_reading - h;

//     BLA::Matrix<3, 3> S;
//     BLA::Matrix<19, 3> K;
//     BLA::Matrix<19, 3> H_t = ~H;
    
//     S = H * oldP * H_t + R;
//     K = (oldP * H_t) * BLA::Inverse(S);
//     BLA::Matrix<19, 1> postErrorState = K * residual;

//     // Inject error angles into quat
//     BLA::Matrix<3, 1> alpha;
//     alpha = {postErrorState(0), postErrorState(1), postErrorState(2)};
//     BLA::Matrix<3, 1> rotVec = 1.0f * alpha;
//     float rotVecNorm = BLA::Norm(rotVec);
//     BLA::Matrix<3,1> axis = rotVec / rotVecNorm;
//     BLA::Matrix<4,1> dq =
//     {
//         cos(rotVecNorm/2.0f),
//         axis(0) * sinf(rotVecNorm/2.0f),
//         axis(1) * sinf(rotVecNorm/2.0f),
//         axis(2) * sinf(rotVecNorm/2.0f),
//     };
//     BLA::Matrix<4, 1> old_q = 
//     {
//         x(0),
//         x(1),
//         x(2),
//         x(3)
//     };
//     BLA::Matrix<4, 1> q = QuaternionUtils::quatMultiply(old_q, dq);

//     // Set quats
//     x(0) = q(0);
//     x(1) = q(1);
//     x(2) = q(2);
//     x(3) = q(3);

//     // Set velocity
//     x(4) = x(4) + postErrorState(3);
//     x(5) = x(5) + postErrorState(4);
//     x(6) = x(6) + postErrorState(5);

//     // Set position
//     x(7) = x(7) + postErrorState(6);
//     x(8) = x(8) + postErrorState(7);
//     x(9) = x(9) + postErrorState(8);

//     // Set gyro bias
//     x(10) = x(10) + postErrorState(9);
//     x(11) = x(11) + postErrorState(10);
//     x(12) = x(12) + postErrorState(11);

//     // Set accel bias
//     x(13) = x(13) + postErrorState(12);
//     x(14) = x(14) + postErrorState(13);
//     x(15) = x(15) + postErrorState(14);

//     // Set mag bias
//     x(16) = x(16) + postErrorState(15);
//     x(17) = x(17) + postErrorState(16);
//     x(18) = x(18) + postErrorState(17);

//     // Set baro bias
//     x(19) = x(19) + postErrorState(18);

//     return x;
// }

