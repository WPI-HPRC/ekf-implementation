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

//ASM libs
#include <ASM330LHHSensor.h>

static ASM330LHHSensor asmimu(&Wire);
#include "QuaternionUtils.h"

using namespace QuaternionUtils;
// This line is a game changer


struct Sample {
  float icm_ax, icm_ay, icm_az;
  float icm_gx, icm_gy, icm_gz;
  int32_t asm_ax, asm_ay, asm_az;
  int32_t asm_gx, asm_gy, asm_gz;
};

static inline void zeroSample(Sample &s) {
  s.icm_ax = s.icm_ay = s.icm_az = 0;
  s.icm_gx = s.icm_gy = s.icm_gz = 0;
  s.asm_ax = s.asm_ay = s.asm_az = 0;
  s.asm_gx = s.asm_gy = s.asm_gz = 0;
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

  //grab icm data
  sensors_event_t accel, gyro, temp, mag;
  icm.getEvent(&accel, &gyro, &temp, &mag);

  s.icm_ax = accel.acceleration.x;
  s.icm_ay = accel.acceleration.y;
  s.icm_az = accel.acceleration.z;

  s.icm_gx = gyro.gyro.x;
  s.icm_gy = gyro.gyro.y;
  s.icm_gz = gyro.gyro.z;

  //gram asm data
  int32_t acc[3] = {0,0,0};
  int32_t gyr[3] = {0,0,0};

  asmimu.Get_X_Axes(acc);
  asmimu.Get_G_Axes(gyr);

  s.asm_ax = acc[0];
  s.asm_ay = acc[1];
  s.asm_az = acc[2];
  s.asm_gx = gyr[0];
  s.asm_gy = gyr[1];
  s.asm_gz = gyr[2];

  return true;
}

BLA::Matrix<3, 1> gyro = {0, 0, 0};
/*
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
*/
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
    DBG.println("w,i,j,k,vel_x,vel_y,vel_z,pos_x,pos_y,pos_z");

}

void loop() {

    if (state == 0) {
        if (get_elapsed_seconds() > 5) {
            state = 1;
        }
        //estimator.padLoop(get_accel(), get_mag(), get_gps_pos());
    } else if (state == 1) {
        estimator.computeInitialOrientation();

        state = 2;
    } else if (state == 2) {
        static uint32_t index = 0;
        float seconds = get_elapsed_seconds();
        if (seconds - lastCalcTimes(0, 0) >= runRates(0, 0)) {
            Sample s;
            readSensorData(s);
            BLA::Matrix<3, 1> gyro = {s.asm_gx, s.asm_gy, s.asm_gz};
            estimator.fastGyroProp(gyro, seconds);
            estimator.ekfPredict(seconds);
        }

        DBG.print(index); DBG.print(',');  
        
        BLA::Matrix<20,1> state = estimator.getState();

        DBG.print("Gyro x: ");DBG.print(gyro(0, 0)); DBG.println(',');
        DBG.print("Gyro y: ");DBG.print(gyro(1, 0)); DBG.print(',');
        DBG.print("Gyro z: ");DBG.print(gyro(2, 0)); DBG.print(',');
        /*
        for (int i = 0; i < 10; i++) {
            DBG.print(state(i, 0)); DBG.print(',');
        }

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

            */

        index++;
    }
    // Poll each sensor
    // Feed to EKF
    // Print quat, vel, pos, biases even
    // Print P diag vector
}
