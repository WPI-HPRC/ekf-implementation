#include <Arduino.h>
#include <Wire.h>

#if defined(USBCON)
#define DBG SerialUSB
#else
#define DBG Serial
#endif

#include "BasicLinearAlgebra.h"
#include "qmekf.h"

#include "QuaternionUtils.h"

using namespace QuaternionUtils;
// This line is a game changer

struct Sample {
  float w, i, j, k;
  float vel_x, vel_y, vel_z;
  float pos_x, pos_y, pos_z;
};

int state = 0;
StateEstimator estimator;
BLA::Matrix<6, 1> lastCalcTimes = {0, 0, 0, 0, 0, 0};
// Gyro, Accel int, Accel up, mag up, gps up, baro up
BLA::Matrix<6, 1> runRates = {0.025, 0.03, 0.03, 0.5, 1, 1};

BLA::Matrix<3, 1> get_gyro() {
    BLA::Matrix<3, 1> gyro = {0, 0, 0};
    return gyro;
}

BLA::Matrix<3, 1> get_accel() {
    BLA::Matrix<3, 1> accel = {0, 0, 0};
    return accel;
}

BLA::Matrix<3, 1> get_mag() {
    BLA::Matrix<3, 1> mag = {0, 0, 0};
    return mag;
}

BLA::Matrix<3, 1> get_gps_pos() {
    BLA::Matrix<3, 1> gps_pos = {0, 0, 0};
    return gps_pos;
}

float get_elapsed_seconds() {
    return millis() / 1000.0f;
}

void setup() {
    // Initialize EKF
    estimator = StateEstimator();
    BLA::Matrix<3, 1> ecef = {0, 0, 0};

    estimator.init(ecef, get_elapsed_seconds());



    // TODO initialize sensors

    DBG.begin(115200);
    delay(5000);

    DBG.println("READY");
    DBG.println("w,i,j,k,vel_x,vel_y,vel_z,pos_x,pos_y,pos_z");

}

void loop() {

    if (state == 0) {
        if (get_elapsed_seconds() > 5) {
            state = 1;
        }
        estimator.padLoop(get_accel(), get_mag(), get_gps_pos());
    } else if (state == 1) {
        estimator.computeInitialOrientation();

        state = 2;
    } else if (state == 2) {
        static uint32_t index = 0;
        float seconds = get_elapsed_seconds();
        if (seconds - lastCalcTimes(0, 0) >= runRates(0, 0)) {
            estimator.fastGyroProp(get_gyro(), seconds);
            estimator.ekfPredict(seconds);
        }

        DBG.print(index); DBG.print(',');  
        
        BLA::Matrix<20,1> state = estimator.getState();
        /*
        for (int i = 0; i < 10; i++) {
            DBG.print(state(i, 0)); DBG.print(',');
        }
            */
        DBG.print("Q1: "); DBG.print(state(0, 0)); DBG.println(',');
        DBG.print("Qx: "); DBG.print(state(1, 0)); DBG.println(',');
        DBG.print("Qy: "); DBG.print(state(2, 0)); DBG.println(',');
        DBG.print("Qz: "); DBG.print(state(3, 0)); DBG.println(',');
        DBG.print("Vx: "); DBG.print(state(4, 0)); DBG.println(',');
        DBG.print("Vy: "); DBG.print(state(5, 0)); DBG.println(',');
        DBG.print("Vz: "); DBG.print(state(6, 0)); DBG.println(',');
        DBG.print("Px: "); DBG.print(state(7, 0)); DBG.println(',');
        DBG.print("Py: "); DBG.print(state(8, 0)); DBG.println(',');
        DBG.print("Pz: "); DBG.print(state(9, 0)); DBG.println(',');



        index++;
    }
    // Poll each sensor
    // Feed to EKF
    // Print quat, vel, pos, biases even
    // Print P diag vector
}
