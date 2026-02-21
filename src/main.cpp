#include <Arduino.h>
#include <Wire.h>

#if defined(USBCON)
#define DBG SerialUSB
#else
#define DBG Serial
#endif


static constexpr uint8_t SDA_PIN = 22;
static constexpr uint8_t SCL_PIN = 23;

#include "BasicLinearAlgebra.h"
#include "qmekf.h"

//ICM20948 libs
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

Adafruit_ICM20948 icm;

//ICM ic2 address
static constexpr uint8_t ICM_ADDR = 0x68;

#include <Adafruit_LPS2X.h>
Adafruit_LPS22 lps;

#include <SparkFun_u-blox_GNSS_v3.h>
SFE_UBLOX_GNSS GPS;

//ASM libs
#include <ASM330LHHSensor.h>

static ASM330LHHSensor asmimu(&Wire);
#include "QuaternionUtils.h"

using namespace QuaternionUtils;
// This line is a game changer

int diddy = 0;

struct MAX10SData {
    float lat;
    float lon;
    float ecefX;
    float ecefY;
    float ecefZ;
    float altMSL;
    float altEllipsoid;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    uint32_t epochTime;
    uint8_t satellites;
    uint8_t gpsLockType;
};

//Sample struct
struct Sample {
  float icm_ax, icm_ay, icm_az;
  float icm_gx, icm_gy, icm_gz;
  float icm_mx, icm_my, icm_mz;
  float asm_ax, asm_ay, asm_az;
  float asm_gx, asm_gy, asm_gz;
  float lps_p, lps_t;
  MAX10SData max10s;
};

static inline void zeroSample(Sample &s) {
  s.icm_ax = s.icm_ay = s.icm_az = 0.0f;
  s.icm_gx = s.icm_gy = s.icm_gz = 0.0f;
  s.icm_mx = s.icm_my = s.icm_mz = 0.0f;
  s.asm_ax = s.asm_ay = s.asm_az = 0.0f;
  s.asm_gx = s.asm_gy = s.asm_gz = 0.0f;
  s.lps_p = s.lps_t = 0.0f;
  s.max10s = {0};
}

struct State {
  float w, i, j, k;
  float vel_x, vel_y, vel_z;
  float pos_x, pos_y, pos_z;
};

int state = 0;
StateEstimator estimator;
BLA::Matrix<6, 1> lastCalcTimes = {0, 0, 0, 0, 0, 0};
// Gyro, Accel int, Accel up, mag up, gps up, baro up
BLA::Matrix<6, 1> runRates = {0.025, 0.03, 0.03, 0.5, 1, 1};

//Read sensor data
static bool readSensorData(Sample &s) {
  zeroSample(s);

  //grab gps data
  s.max10s.lat = (float)GPS.getLatitude();
  s.max10s.lon = (float)GPS.getLongitude();
  s.max10s.velN = GPS.getNedNorthVel();
  s.max10s.velE = GPS.getNedEastVel();
  s.max10s.velD = GPS.getNedDownVel();
    /** 
  s.max10s.ecefX = (float)GPS.getHighResECEFX() * 0.01f;
  s.max10s.ecefY = (float)GPS.getHighResECEFY() * 0.01f;
  s.max10s.ecefZ = (float)GPS.getHighResECEFZ() * 0.01f;
  s.max10s.altMSL = (float)GPS.getAltitudeMSL() / 1000.0f;
  s.max10s.altEllipsoid = (float)GPS.getAltitude() / 1000.0f;
  s.max10s.epochTime = GPS.getUnixEpoch();
  s.max10s.satellites = GPS.getSIV();
  s.max10s.gpsLockType = GPS.getFixType();  
  */



  //grab lps data
  sensors_event_t pressure, lps_temp;
  lps.getEvent(&pressure, &lps_temp);
  s.lps_p = pressure.pressure;
  s.lps_t = lps_temp.temperature;

  //grab icm data
  sensors_event_t accel, gyro, temp, mag;
  icm.getEvent(&accel, &gyro, &temp, &mag);

  s.icm_ax = accel.acceleration.x * 0.980665f;
  s.icm_ay = accel.acceleration.y * 0.980665f;
  s.icm_az = accel.acceleration.z * 0.980665f;

  s.icm_gx = gyro.gyro.x * (PI / 180.0f);
  s.icm_gy = gyro.gyro.y * (PI / 180.0f);
  s.icm_gz = gyro.gyro.z * (PI / 180.0f);

  s.icm_mx = mag.magnetic.x;
  s.icm_my = mag.magnetic.y;
  s.icm_mz = mag.magnetic.z;

  //gram asm data
  int32_t acc[3] = {0,0,0};
  int32_t gyr[3] = {0,0,0};

  asmimu.Get_X_Axes(acc);
  asmimu.Get_G_Axes(gyr);

  s.asm_ax = acc[0] * 0.00980665f;
  s.asm_ay = acc[1] * 0.00980665f;
  s.asm_az = acc[2] * 0.00980665f;
  s.asm_gx = gyr[0] / 1000.0f * (PI / 180.0f);
  s.asm_gy = gyr[1] / 1000.0f * (PI / 180.0f);
  s.asm_gz = gyr[2] / 1000.0f * (PI / 180.0f);

  return true;
}

BLA::Matrix<3, 1> gyro = {0, 0, 0};
BLA::Matrix<3, 1> accel = {0, 0, 0};
BLA::Matrix<3, 1> mag = {0, 0, 0};
BLA::Matrix<1, 1> baro = {0};
BLA::Matrix<2, 1> gps_pos = {0, 0};
BLA::Matrix<3, 1> gps_vel = {0, 0, 0};

float get_elapsed_seconds() {
    return millis() / 1000.0f;
}

void setup() {

    DBG.begin(115200);
    delay(6000);


    //I2C
    Wire.setSDA(PB_7);
    Wire.setSCL(PB_6);
    Wire.begin();
    Wire.setClock(100000);
    delay(50);

    //init gps
    if (GPS.begin()) {
    GPS.setNavigationRate(1);
    GPS.setAutoPVT(true);
    Serial.println("GPS init OK");
    } 
    else {
      Serial.println("GPS init FAILED");
    }

    //init baro
    if(!lps.begin_I2C(0x5C)) {
        DBG.println("LPS init failed");
    } else {
        DBG.println("LPS init OK");
    }

    lps.setDataRate(LPS22_RATE_50_HZ);

    //ASM init
    int asmStatus = asmimu.begin();
    if (asmStatus != 0) {
        DBG.print("ASM330 init failed: ");
        DBG.println(asmStatus);
    } else {
        DBG.println("ASM330 init OK");
        asmimu.Enable_X();
        asmimu.Enable_G();
    }

    //ICM init
    if (!icm.begin_I2C(0x68, &Wire)) {
        DBG.println("ICM init failed");
    } else {
        DBG.println("ICM init OK");
    }

    icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
    icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
    icm.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);

    //Configure ICM
    icm.setAccelRange(ICM20948_ACCEL_RANGE_4_G);
    icm.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);

    icm.setAccelRateDivisor(0);  
    icm.setGyroRateDivisor(0);    

    // Initialize EKF
    estimator = StateEstimator();
    BLA::Matrix<3, 1> ecef = {0, 0, 0};

    estimator.init(ecef, get_elapsed_seconds());
    // TODO initialize sensors

    DBG.begin(115200);
    delay(5000);

    DBG.println("READY");
    DBG.println("index,t,w,i,j,k,vel_x,vel_y,vel_z,pos_x,pos_y,pos_z");

}

void loop() {
    /*
    if (state == 0) {
        if (get_elapsed_seconds() > 5) {
            state = 1;
        }
        //estimator.padLoop(get_accel(), get_mag(), get_gps_pos());
    } else if (state == 1) {
        estimator.computeInitialOrientation();

        state = 2;
    } else if (state == 2) {
     */
        static uint32_t index = 0;
        float seconds = get_elapsed_seconds();
        if (seconds - lastCalcTimes(0, 0) >= runRates(0, 0)) {
            lastCalcTimes(0, 0) = seconds;
            Sample s;
            readSensorData(s);
            gyro = {s.asm_gx, s.asm_gy, s.asm_gz};
            accel = {s.asm_ax, s.asm_ay, s.asm_az};
            mag = {s.icm_mx, s.icm_my, s.icm_mz};
            baro = {s.lps_p};

            
            estimator.fastGyroProp(gyro, seconds);
            estimator.fastAccelProp(accel, seconds);
            estimator.ekfPredict(seconds);

            estimator.runAccelUpdate(accel, seconds);
            estimator.runMagUpdate(mag, seconds);
            //estimator.runBaroUpdate(baro, seconds);
            
            //DBG.print("Gyro x: ");DBG.print(gyro(0, 0)); DBG.println(',');
            //DBG.print("Gyro y: ");DBG.print(gyro(1, 0)); DBG.println(',');
            //DBG.print("Gyro z: ");DBG.print(gyro(2, 0)); DBG.println(',');
            diddy++;
        }
        //DBG.print(index); DBG.println(',');  
        
        BLA::Matrix<20,1> state = estimator.getState();


        /*
        for (int i = 0; i < 10; i++) {
            DBG.print(state(i, 0)); DBG.print(',');
        }
                */
        DBG.print(index); DBG.print(',');
        DBG.print(get_elapsed_seconds()); DBG.print(',');
        //DBG.print("Q1: ");
        DBG.print(state(0, 0)); DBG.print(',');
        //DBG.print("Qx: ");
        DBG.print(state(1, 0)); DBG.print(',');
        //DBG.print("Qy: ");
        DBG.print(state(2, 0)); DBG.print(',');
        //DBG.print("Qz: ");
        DBG.print(state(3, 0)); DBG.print(',');
        
        //DBG.print("Vx: "); 
        DBG.print(state(4, 0)); DBG.print(',');
        //DBG.print("Vy: "); 
        DBG.print(state(5, 0)); DBG.print(',');
        //DBG.print("Vz: "); 
        DBG.print(state(6, 0)); DBG.print(',');
        //DBG.print("Px: "); 
        DBG.print(state(7, 0)); DBG.print(',');
        //DBG.print("Py: "); 
        DBG.print(state(8, 0)); DBG.print(',');
        //DBG.print("Pz: "); 
        DBG.println(state(9, 0));

        index++;
    //}
    // Poll each sensor
    // Feed to EKF
    // Print quat, vel, pos, biases even
    // Print P diag vector
}
