#include "vp_kf.h"

PVStateEstimator::PVStateEstimator(){}

void PVStateEstimator::init(BLA::Matrix<6,1> initial, float dt){
    initialPV = initial; 
    x = initial; 
    this.dt = dt; 
}

/*Converts accel x,y,z readings from Body frame to NED*/
BLA::Matrix<3,1> body2ned(BLA::Matrix<4,1> orientation, float accelX, float accelY, float accelZ){

    // Set quaternion values 
    float qw = orientation(0); 
    float qx = orientation(1); 
    float qy = orientation(2); 
    float qz = orientation(3); 

    BLA::Matrix<3, 3> rotm = {
        qw*qw + qx*qx - qy*qy - qz*qz, 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy),
        2 * (qx * qy + qw * qz), qw*qw - qx*qx + qy*qy - qz*qz, 2 * (qy * qz - qw * qx),
        2 * (qx * qz - qw * qy), 2 * (qw * qx + qy * qz), qw*qw - qx*qx - qy*qy + qz*qz
    };

    g = {0,0,-9.81}; 
    BLA::Matrix<3,1> accel_body = {accelX, accelY, accelZ}; 
    BLA::Matrix<3,1> accel_ned = accel_body * rotm; 
    return accel_ned - g; //account for gravity, assumes accel in m/s^2

}

/*Converts predicted state from lla to ECEF*/
BLA::Matrix<3,1> lla2ecef(BLA::Matrix<3,1> lla){

    float lat = lla(0) * pi / 180.0; // Convert to radians 
    float lon = lla(1) * pi / 180.0; // Convert to radians 
    float alt = lla(2); 

    // Convert reference lla to ecef first 
    double N = a / std::sqrt(1 - e2 * std::pow(std::sin(lat), 2));

    double x = (N + alt) * std::cos(lat) * std::cos(lon);
    double y = (N + alt) * std::cos(lat) * std::sin(lon);
    double z = ((1 - e2) * N + alt) * std::sin(lat);

    BLA::Matrix<3,1> ecef_coords = {x,y,z}; 

    return ecef_coords; 

} 

BLA::Matrix<3,3> getRotM(BLA::Matrix<3,1> lla){
    float lat = lla(0) * pi / 180.0; // Convert to radians 
    float lon = lla(1) * pi / 180.0; // Convert to radians 
    return{
         {-std::sin(lat) * std::cos(lon), -std::sin(lon), -std::cos(lat) * std::cos(lon),
        -std::sin(lat) * std::sin(lon),  std::cos(lon), -std::cos(lat) * std::sin(lon),
         std::cos(lat),                 0.0,            -std::sin(lat)};
    }
}

/*Converts predicted state from NED to lla*/
BLA::Matrix<6,1> ned2ecef(BLA::Matrix<6,1> state, Utility::TelemPacket sensorPacket){ // change so just gps readings 
    BLA::Matrix<3,1> ref_lla = {initialPV(0),initialPV(1),initialPV(2)}; 
    BLA::Matrix<3,1> ref_ecef = lla2ecef(ref_lla);

    BLA::Matrix<3,3> R = getRotM(ref_lla); 

    BLA::Matrix<3,1> pos = {state(0), state(1), state(2)}; 
    BLA::Matrix<3,1> ecef_pos = ref_ecef + R*pos; 

    BLA::Matrix<3,1> vel = {state(3), state(4), state(5)};   
    BLA::Matrix<3,1> currGPS = {sensorPacket.lat, sensorPacket.lon, 0}; 
    R = getRotM(curr_GPS); 
     
    BLA::Matrix<3,1> ecef_vel = vel*R; 

    BLA::Matrix<6,1> x_ecef = {ecef_pos(0), ecef_pos(1), ecef_pos(2),vel_pos(0),vel_pos(1), vel_pos(2)}; 
    
    return x_ecef; 

} 
    

BLA::Matrix<6,1> onLoop(Utility::TelemPacket sensorPacket){

    //Convert accel readings from body2ned, depending on quat rep will need to change this 
    BLA::Matrix<3,1> accel_ned = body2ned(orientation, sensorPacket.accelX, sensorPacket.accelY, sensorPacket.accelZ); // This will need to be fixed to align with current sensor stuff, also check correct units
    
    /*Prediction Step*/
    // Use the measurement model to predict the next state 
    BLA::Matrix<6,1> x_ned = F*x + B*u; // Noise needs to be added here (w matrix)
    x = ned2lla(x_ned); 

    // Propogate Covariance
    P = F * P * BLA::MatrixTranspose<BLA::Matrix<6, 6>>(F) + Q; // This covariance currently doesn't change at all? 

    if(/*last time read logic*/){ 

       // Convert barometer to correct units 
        float alt = sensorPacket.alt; 
        // Compile z Matrix 
        BLA::Matrix<3,1> z = {sensorPacket.lat, sensorPacket.lon, alt}; 
        // Now do the update step
        this.x = updateState(z);    
    }
   
    x = lla2ecef(x); 
    return x; 
}


BLA::Matrix<6,1> updateState(BLA::Matrix<3,1> z){ //H dimension should change prob

    // Compute the innovation 
    BLA::Matrix<3,1> y = z - H*x; // Add Observation Noise

    // Compute S matrix 
    BLA::Matrix<3,3> S = H*P*BLA::MatrixTranspose<BLA::Matrix<3, 6>>(H) + R; 

    // Compute the Kalman Gain 
    K = P*BLA::MatrixTranspose<BLA::Matrix<3, 6>>(H)*BLA::inverse(S); 

    //Compute updated state
    BLA::Matrix<6,1> updated_x = x + K*y; 

    // Update state covariance
    P = (I - K*H)*P; 

    return updated_x; 
}






