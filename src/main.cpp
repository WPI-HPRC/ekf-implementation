#include <Arduino.h>

#include "BasicLinearAlgebra.h"

#include "QuaternionUtils.h"

void setup() {
    // Since basing this whole c++ impl on the simulink, using that as reference here
    Serial.println("Hello");

    BLA::Matrix<4, 1> random_quat = {0.9344, 0.2435, 0.2435, 0.0907};
    BLA::Matrix<4, 1> random_quat2 = {0.3666, 0.4564, 0.7857, 0.1998};
    BLA::Matrix<3, 3> random_mat = {0.8662, 0.0206,0.2123, 0.6011, 0.9699, 0.1818, 0.7081, 0.8324, 0.1834};
    BLA::Matrix<3, 1> random_vec = {0.3042, 0.5248, 0.4319};
    BLA::Matrix<3, 1> school_lla = {42.274027, -71.811788, 10};

    Serial.println("Test quat2dcm: Quat: {0.9344, 0.2435, 0.2435, 0.0907}. Expected DCM: \
    0.8650    0.2880   -0.4109 \
   -0.0508    0.8649    0.4993 \
    0.4992   -0.4110    0.7628. Actual DCM:");

    Serial.println(QuaternionUtils::quat2DCM(random_quat));



    Serial.println("Test skewSymmetric: Random vec: {0.3042, 0.5248, 0.4319}. Expected Mat: \
        0   -0.4319    0.5248 \
    0.4319         0   -0.3042 \
   -0.5248    0.3042         0");

   Serial.println(QuaternionUtils::skewSymmetric(random_vec));



   Serial.println("Test rotVec2Quat: Random vec: {0.3042, 0.5248, 0.4319}. Expected Quat: {0.9315, 0.2769, 0.4776, 0.3931}");

   Serial.println(QuaternionUtils::rotVec2Quat(random_vec));



   Serial.println("Test smallAngleRotVec2Quat: Random vec: {0.3042, 0.5248, 0.4319}. Expected Quat: {1.0000, 0.1521, 0.2624, 0.2160}");

   Serial.println(QuaternionUtils::smallAngleRotVec2Quat(random_vec));



//    TODO eventually test dcm2quat


    Serial.println("Test quatMultiply: Random Quats: {0.9344    0.2435    0.2435    0.0907} and {0.3666    0.4564    0.7857    0.1998} \
        Expected Quat: {0.0220    0.4932    0.8162    0.3002}");

    Serial.println(QuaternionUtils::quatMultiply(random_quat, random_quat2));


    Serial.println("Test lla2ecef: School LLA: {42.274027, -71.811788, 10}. Expected ECEF: {1475354, -4490428, 4268181}");

    Serial.println(QuaternionUtils::lla2ecef(school_lla));


    Serial.println("Test dcm_ned2ecef: School LLA: {42.274027, -71.811788, 10}. Expected DCM: \
        -0.2100    0.9500   -0.2310 \
        0.6391    0.3121    0.7030 \
        0.7399         0   -0.6727");
    Serial.println(QuaternionUtils::dcm_ned2ecef(school_lla));


    Serial.println("Test ecef2nedVec:"); // Eh one day

    Serial.println("Test quatConj: Test quat: {0.9344    0.2435    0.2435    0.0907}. Expected quat: 0.9344   -0.2435   -0.2435   -0.0907");
    Serial.println(QuaternionUtils::quatConjugate(random_quat));

    BLA::Matrix<3, 1> v1_b = {-8.4870, -4.7330, -1.2682};
    BLA::Matrix<3, 1> v2_b = {-0.1710, 0.0429, 0.9843};
    BLA::Matrix<3, 1> v1_i = {0, 0, 9.8};
    BLA::Matrix<3, 1> v2_i = {1, 0, 0};

    Serial.println("Test triad_BE: Test vecs displayed here in code. Expected: \
        -0.1710   -0.4698   -0.8660 \
        0.0429    0.8746   -0.4830 \
        0.9843   -0.1197   -0.1294");
    Serial.println(QuaternionUtils::triad_BE(v1_b, v2_b, v1_i, v2_i));
    

}

void loop() {
}