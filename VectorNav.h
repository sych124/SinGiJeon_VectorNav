/*
VectorNav.h
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2017 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef VECTORNAV_h
#define VECTORNAV_h

#include "Arduino.h"
// #include "digitalWriteFast.h"
#include "SPI.h"

typedef struct Gps{
    double tow;
    uint16_t week;
    uint8_t FixType, NumSV;

    double PosX, PosY, PosZ;
    float VelX, VelY, VelZ;

    float NorthAcc, EastAcc, VertAcc, SpeedAcc, TimeAcc;
}GPSStruct;

typedef struct Ins{
    double tow;
    uint16_t week;
    uint8_t mode, Fix, ErrorType;

    float yaw, pitch, roll;
    double PosX, PosY, PosZ;
    float VelX, VelY, VelZ;

    float AttAcc, PosAcc, VelAcc;
}INSStruct;

typedef struct Nav{
    float yaw, pitch, roll;
    double PosX, PosY, PosZ;
    float VelX, VelY, VelZ;
    float ax,ay,az;
    float gx, gy, gz;
}NAVStruct;


class VectorNav{
  public:
    VectorNav(uint8_t cspin);
    VectorNav(uint8_t csPin, SPIClass *Spi);
    int begin();
    
    int enableInterrupt(uint16_t SRD, uint32_t pulseWidth);
    int setDLPF(uint16_t magWindowSize, uint16_t accelWindowSize, uint16_t gyroWindowSize, uint16_t temperatureWindowSize, uint16_t pressureWindowSize);
    int setReferenceFrameRotation(float T[3][3]);
    
    int getAccel(float* ax, float* ay, float* az);
    int getGyro(float* gx, float* gy, float* gz);
    int getMag(float* hx, float* hy, float* hz);
    int getPressure(float* pressure);
    int getTemperature(float* temperature);
    int getMotion6(float* ax, float* ay, float* az, float* gx, float* gy, float* gz);
    int getMotion9(float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* hx, float* hy, float* hz);
    int getEuler(float* yaw, float* pitch, float* roll);
    int getEulerIMU(float* yaw, float* pitch, float* roll, float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* hx, float* hy, float* hz);
    int getQuat(float* quat[4]);
    int getQuatIMU(float* quat[4], float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* hx, float* hy, float* hz);
    int getEulerGyroAccel(float* yaw, float* pitch, float* roll, float* InertialAccelX, float* InertialAccelY, float* InertialAccelZ, float* GyroX, float* GyroY, float* GyroZ);
    int getVel(float* VelX, float* VelY, float* VelZ);

    void writeSettings();
    void restoreSettings();
    void resetSensor();
    int readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
    int writeRegisters(uint8_t subAddress, uint8_t count, uint8_t* buffer);

    
 

    // spi
    uint8_t _csPin;
    SPIClass *_spi;
    bool _useSPI;
    const uint32_t SPI_CLOCK = 16000000; // 16 MHz

    // timing, used to ensure at least 50us between communication with VN
    uint32_t timeSinceTX;

    // conversions
    const float G2uT = 100.0f;
    const float kPa2Pa = 1000.0f;
    const float deg2rad = PI/180.0f;
    const double deg2radL = 3.141592653589793238462643383279502884L/180.0L;

    // VectorNav communication header length
    const uint8_t HEADER_LENGTH = 4;

    // commands
    const uint8_t CMD_READ = 0x01;
    const uint8_t CMD_WRITE = 0x02;
    const uint8_t CMD_FLASH = 0x03;
    const uint8_t CMD_RESTORE = 0x04;
    const uint8_t CMD_RESET = 0x06;
    const uint8_t CMD_MAG_DISTURBANCE = 0x08;
    const uint8_t CMD_ACCEL_DISTURBANCE = 0x09;
    const uint8_t CMD_SET_GYRO_BIAS     = 0x0C;

    /* registers and sizes (register number, number of bytes) */

    // Device configuration registers
    const uint8_t USER_TAG_REG[2]                   = {0,   20};
    const uint8_t MODEL_NUMBER_REG[2]               = {1,   24};
    const uint8_t HARDWARE_REV_REG[2]               = {2,   4};
    const uint8_t SERIAL_NUM_REG[2]                 = {3,   4};
    const uint8_t FIRMWARE_VERSION_REG[2]           = {4,   4};
    const uint8_t SERIAL_BAUDRATE_REG[2]            = {5,   4};
    const uint8_t ASYNC_DATA_OUTPUT_TYPE_REG[2]     = {6,   4};
    const uint8_t ASYNC_DATA_OUTPUT_FREQ_REG[2]     = {7,   4};
    const uint8_t SYNC_CONTROL_REG[2]               = {32,  20};
    const uint8_t COMM_PROTOCOL_CNTRL_REG[2]        = {30,  7};
    const uint8_t BIN_OUTPUT_1_REG[2]               = {75,  22};
    const uint8_t BIN_OUTPUT_2_REG[2]               = {76,  22};
    const uint8_t BIN_OUTPUT_3_REG[2]               = {77,  22};

    // IMU configuration
    const uint8_t MAG_COMPENSATION_REG[2]           = {23,  48};     
    const uint8_t ACCEL_COMPENSATION_REG[2]         = {25,  48};
    const uint8_t GYRO_COMPENSATION_REG[2]          = {84,  48};
    const uint8_t REF_FRAME_ROTATION_REG[2]         = {26,  36};
    const uint8_t IMU_FILTER_CONFIG_REG[2]          = {85,  15};
    const uint8_t DELTA_THETA_VEL_CONFIG_REG[2]     = {82,  6};

    // Attitude configuration
    const uint8_t VPE_BASIC_CNTRL_REG[2]            = {35,  4};
    const uint8_t VPE_MAG_TUNING_REG[2]             = {36,  36};
    const uint8_t VPE_ACCEL_TUNING_REG[2]           = {38,  36};

    // Hard/soft iron estimator configuration
    const uint8_t MAG_HSI_CNTRL_REG[2]              = {44,  4};

    // World magnetic and gravity model configuration
    const uint8_t MAG_GRAV_REFERENCE_REG[2]         = {21,  24};
    const uint8_t REF_VECTOR_CONFIGURATION_REG[2]   = {83,  32};

    // Hard/soft iron estimator status
    const uint8_t CALC_HSI_REG[2]                   = {47,  48};    

    // Device status registers
    const uint8_t SYNC_STATUS_REG[2]                = {33,  12};

    // IMU measurements
    const uint8_t IMU_MEAS_REG[2]                   = {54,  44};
    const uint8_t DELTA_THETA_VEL_REG[2]            = {80,  28};

    // Attitude measurements
    const uint8_t EULER_REG[2]                      = {8,   12};
    const uint8_t QUATERNION_REG[2]                 = {9,   16};
    const uint8_t EULER_IMU_REG[2]                  = {27,  48};
    const uint8_t QUATERNION_IMU_REG[2]             = {15,  52};
    const uint8_t MAG_COMP_REG[2]                   = {17,  12};
    const uint8_t ACCEL_COMP_REG[2]                 = {18,  12};
    const uint8_t GYRO_COMP_REG[2]                  = {19,  12};
    const uint8_t IMU_COMP_REG[2]                   = {20,  36};
    const uint8_t EULER_GYRO_BODY_ACCEL_REG[2]      = {239, 36};
    const uint8_t EULER_GYRO_INERTIAL_ACCEL_REG[2]  = {240, 36};

    // Velocity compensation input
    const uint8_t VEL_COMPENSATION_MEAS_REG[2]      = {50,  12};
};

class VN100: public VectorNav {
  public:
    using VectorNav::VectorNav;
    int velocityCompensation(float U, float V, float W);
    void tareAttitude();
  private:
    // commands
    const uint8_t CMD_TARE = 0x05;

    // Velocity compensation configuration
    const uint8_t VEL_COMPENSATION_CNTRL_REG[2]     = {51,  8};

    // Velocity compensation input
    const uint8_t VEL_COMPENSATION_MEAS_REG[2]      = {50,  12};
};

class VN200: public VectorNav {
  public:
    using VectorNav::VectorNav;
    int setAntennaOffset(float positionX, float positionY, float positionZ);

    void setFilterBias();

    int getGpsLla(double* tow, uint16_t* week, uint8_t* FixType, uint8_t* NumSV, double* latitude, double* longitude, double* altitude, float* NEDVelX, float* NEDVelY, float* NEDVelZ, float* NorthAcc, float* EastAcc, float* VertAcc, float* SpeedAcc, float* TimeAcc);
    int getGpsEcef(double* tow, uint16_t* week, uint8_t* FixType, uint8_t* NumSV, double* PosX, double* PosY, double* PosZ, float* VelX, float* VelY, float* VelZ, float* XAcc, float* YAcc, float* ZAcc, float* SpeedAcc, float* TimeAcc);
    int getInsLla(double* tow, uint16_t* week, uint8_t* mode, uint8_t* Fix, uint8_t* ErrorType, float* yaw, float* pitch, float* roll, double* latitude, double* longitude, double* altitude, float* NEDVelX, float* NEDVelY, float* NEDVelZ, float* AttAcc, float* PosAcc, float* VelAcc);
    int getInsEcef(double* tow, uint16_t* week, uint8_t* mode, uint8_t* Fix, uint8_t* ErrorType, float* yaw, float* pitch, float* roll, double* PosX, double* PosY, double* PosZ, float* VelX, float* VelY, float* VelZ, float* AttAcc, float* PosAcc, float* VelAcc);
    int getNavLla(float* yaw, float* pitch, float* roll, double* latitude, double* longitude, double* altitude, float* NEDVelX, float* NEDVelY, float* NEDVelZ, float* ax, float* ay, float* az, float* gx, float* gy, float* gz);
    int getNavEcef(float* yaw, float* pitch, float* roll, double* PosX, double* PosY, double* PosZ, float* VelX, float* VelY, float* VelZ, float* ax, float* ay, float* az, float* gx, float* gy, float* gz);
  private:
    // commands
    const uint8_t CMD_SET_FILTER_BIAS = 0x11;

    // GPS configuration
    const uint8_t GPS_CONFIG_REG[2]                 = {55,  5};
    const uint8_t GPS_ANT_OFFSET_REG[2]             = {57,  12};

    // GPS measurements
    const uint8_t GPS_LLA_MEAS_REG[2]               = {58,  72};
    const uint8_t GPS_ECEF_MEAS_REG[2]              = {59,  72};

    // INS configuration
    const uint8_t INS_CONFIG_REG[2]                 = {67,  4};
    const uint8_t INS_STARTUP_FILTER_BIAS_REG[2]    = {74,  28};

    // INS measurements
    const uint8_t INS_LLA_SOL_MEAS_REG[2]           = {63,  72};
    const uint8_t INS_ECEF_SOL_MEAS_REG[2]          = {64,  72};
    const uint8_t INS_LLA_STATE_MEAS_REG[2]         = {72,  72};
    const uint8_t INS_ECEF_STATE_MEAS_REG[2]        = {73,  72};
};

/* VN-300 firmware still in beta without SPI support. Below is an
educated guess at the additional commands and registers needed.
class VN300: public VN200{
  public:
    using VN200::VN200;
  private:

    // commands
    const uint8_t CMD_SET_INITIAL_HEADING = 0x12;

    // GPS configuration
    const uint8_t GPS_COMPASS_BASELINE_CONFIG_REG[2]= {93,  24};
    const uint8_t GPS_COMPASS_ESTIMATED_BASELINE_REG[2]   = {97,  28};
};
*/

class Publisher: public VN200 {
  public:
    using VN200::VN200;

    GPSStruct GpsLla;
    GPSStruct GpsEcef;
    INSStruct InsLla;
    INSStruct InsEcef;
    NAVStruct NavLla;
    NAVStruct NavEcef;

    int GpsLlaPub(GPSStruct *GpsLla);
    int GpsEcefPub(GPSStruct *GpsEcef);
    int InsLlaPub(INSStruct *InsLla);
    int InsEcefPub(INSStruct *InsEcef);
    int NavLlaPub(NAVStruct *NavLla);
    int NavEcefPub(NAVStruct *NavEcef);
};

#endif
