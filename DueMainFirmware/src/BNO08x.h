/* 
* Sys-MoDEL Active Suspension BNO08x.h file
*
* Authors: Gregory Stewart, John Estafanos, Andrew Kennah, Patrick Laforest
* Creation Date: Jan 6, 2024
* Last Update: March 6, 2024
*
* Version 1.0
*
* Description: 
*
* Functions & Descriptions: 
* Name: 
* Description: 
*
* Name: 
* Description: 
* References:
*
*/

#ifndef BNO08x_H
#define BNO08x_H

#include <Arduino.h>
#include <Adafruit_BNO08x.h>

class BNO08xIMU
{
  public:
    // Constructor 
    BNO08xIMU();

     // Struct for yaw, pitch, and roll
        struct euler_t {
        float _yaw;
        float _pitch;
        float _roll;
        } _ypr;

    // SH2 Report setting protocol function
    void SetReports();

    // Begin
    void BeginBNO08x();

    // Math
    void QuaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees);

    // Math
    void QuaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees);

    // Get gyro data
    bool GetGyroEvent(sh2_SensorValue_t* gyroData);
    float GetGyroRollData();
    float GetGyroPitchData();

    // Get linear acceleration in the z-direction
    bool GetLinearZAccelerationEvent(sh2_SensorValue_t* linearAccelData);
    float GetAccZData();

    // Get pitch and roll data
    bool GetPitchAndRollEvent(sh2_SensorValue_t* pitchAndRollData);
    void GetRollAndPitchData();




  private: 
    // Create object
    Adafruit_BNO08x  *_bno08x;

    sh2_SensorValue_t _gyroValues;
    sh2_SensorValue_t _linearAccelValue;
    sh2_SensorValue_t _pitchAndRollValues;

    // Quaternion Values
    float _i;
    float _j;
    float _k;
    float _real;


    // Variables for calibration
    double _sumYaw = 0.0;
    double _sumPitch = 0.0;
    double _sumRoll = 0.0;

    long _reportIntervalUs = 5000;
    

};

#endif