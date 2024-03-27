/* 
* Sys-MoDEL Active Suspension BNO08x.cpp file
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

#include "BNO08x.h"

BNO08xIMU::BNO08xIMU()
{
    _bno08x = new Adafruit_BNO08x();
}

void BNO08xIMU::SetReports() 
{
    _bno08x->enableReport(SH2_GYROSCOPE_CALIBRATED, 2000);
    _bno08x->enableReport(SH2_LINEAR_ACCELERATION, 2000);
    _bno08x->enableReport(SH2_ARVR_STABILIZED_RV, 2000);
}

void BNO08xIMU::QuaternionToEuler(float qr, float qi, float qj, float qk, euler_t* _ypr, bool degrees)
{
    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    _ypr->_yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    _ypr->_pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    _ypr->_roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      _ypr->_yaw *= RAD_TO_DEG;
      _ypr->_pitch *= RAD_TO_DEG;
      _ypr->_roll *= RAD_TO_DEG;
    }
}

void BNO08xIMU::QuaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* _ypr, bool degrees) 
{
    QuaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, _ypr, degrees);
}

void BNO08xIMU::BeginBNO08x()
{
  if (!_bno08x->begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("BNO08x Found!");   
}

// Functions to get Gyro Data
bool BNO08xIMU::GetGyroEvent(sh2_SensorValue_t* gyroData)
{
  return _bno08x->getSensorEvent(gyroData);
}

float BNO08xIMU::GetGyroRollData()
{
    if(GetGyroEvent(&_gyroValues))
    {
        return _gyroValues.un.gyroscope.x;
    }
}

float BNO08xIMU::GetGyroPitchData()
{
    if(GetGyroEvent(&_gyroValues))
    {
        return _gyroValues.un.gyroscope.y;
    }
}

// Get linear acceleration in the z-direction
bool BNO08xIMU::GetLinearZAccelerationEvent(sh2_SensorValue_t* linearAccelData)
{
  return _bno08x->getSensorEvent(linearAccelData);
}

float BNO08xIMU::GetAccZData()
{
    if(GetLinearZAccelerationEvent(&_linearAccelValue))
    {
        return _linearAccelValue.un.linearAcceleration.z;
    }
}

// Get pitch and roll quaternion data
bool BNO08xIMU::GetPitchAndRollEvent(sh2_SensorValue_t* pitchAndRollData)
{
  return _bno08x->getSensorEvent(pitchAndRollData);
}

void BNO08xIMU::GetRollAndPitchData()
{
    if (GetPitchAndRollEvent(&_pitchAndRollValues))
    {
        return QuaternionToEulerRV(&_pitchAndRollValues.un.arvrStabilizedRV, &_ypr, true);
    }
}