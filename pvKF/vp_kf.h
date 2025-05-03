#pragma once 

#include "Arduino.h"
#include "BasicLinearAlgebra.h"
#include "../../src/utility.hpp"

using namespace BLA; 

class PVStateEstimator{

    public: 
    // Remember to make the necessary variables private...

    PVStateEstimator(); 

    void init(BLA::Matrix<6,1> initial, float dt); //Do I also need initial orientation? 

    void onLoop(Utility::TelemPacket sensorPacket); 
    
    // Global Variables
    // Should these be pointers? 
    const float dt; // Loop time
    const float accelXYZ_var = 60; // Accel X,Y,Z Variance microg's/sqrt(Hz)
    const float gps_var = 0.0; // This may need to be split into different axis, GPS Variance
    const float baro_var = 0.0; // Barometer Variance 

    // Helper Constants 
    constexpr double a = 6378137.0;                // WGS-84 semi-major axis
    constexpr double f = 1.0 / 298.257223563;      // flattening
    constexpr double e2 = f * (2 - f);             // eccentricity squared
    constexpr double pi = 3.14159265358979323846;
   
    // State vector 
    BLA::Matrix<6,1> initialPV; // For reference 
    BLA::Matrix<6,1> x; 

    // Kalman Gain
    BLA::Matrix<6,3> K; 

    // Estimate Covariance 
    BLA::Matrix<6,6> P; 

    // Sensor Covariance Matrices, these will be defined based on sensor variance 
    BLA::Matrix<6,6> Q;
    BLA::Matrix<3,3> R;

    //F and B Matrices, make sure dt is declared beforehand in init or this will fail
    const BLA::Matrix<6,6> F = {
        1,0,0,this.dt,0,0,
        0,1,0,0,this.dt,0,
        0,0,1,0,0,this.dt,
        0,0,0,1,0,0,
        0,0,0,0,1,0,
        0,0,0,0,0,1
    };

    const BLA::Matrix<6,3> B = {
        0.5*this.dt*this.dt, 0, 0,
        0,0.5*this.dt*this.dt,0,
        0,0,0.5*this.dt*this.dt,
        this.dt,0,0,
        0,this.dt,0, 
        0,0,this.dt 
    };

    // Observation Matrix H
    const BLA::Matrix<3,6> H = {
        1,0,0,0,0,0,
        0,1,0,0,0,0,
        0,0,1,0,0,0
    };

    const BLA::Matrix<6,6> I = {
        1,0,0,0,0,0,
        0,1,0,0,0,0,
        0,0,1,0,0,0,
        0,0,0,1,0,0,
        0,0,0,0,1,0,
        0,0,0,0,0,1
    }; 

    

    








}