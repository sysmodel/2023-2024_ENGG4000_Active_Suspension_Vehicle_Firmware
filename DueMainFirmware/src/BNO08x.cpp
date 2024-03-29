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
    _bno08x->enableReport(SH2_GYROSCOPE_CALIBRATED, _reportIntervalUs);
    _bno08x->enableReport(SH2_LINEAR_ACCELERATION, _reportIntervalUs);
    _bno08x->enableReport(SH2_ARVR_STABILIZED_RV, _reportIntervalUs);
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
  //if (!_bno08x->begin_I2C()) {
    //Serial.println("Failed to find BNO08x chip");
    //while (1) {
      //delay(10);
    //}
  //}
  // Serial.println("BNO08x Found!");   
  _bno08x->begin_I2C();
  SetReports();

}

void BNO08xIMU::GetDataIMU()
{
  // Check if reset occured 
  if (_bno08x->wasReset()) 
  {
    // Serial.print("sensor was reset ");
    SetReports();
  }

  // Get data from each sensor event and store in struct
  if(!_bno08x->getSensorEvent(&_sensorValue))
  {
    return;
  }

  switch(_sensorValue.sensorId)
  {
    case SH2_GYROSCOPE_CALIBRATED:
      _rpRates._pitchRate = _sensorValue.un.gyroscope.x;
      _rpRates._rollRate = _sensorValue.un.gyroscope.y;
    break;

    case SH2_LINEAR_ACCELERATION:
      _rpRates._zAcc = _sensorValue.un.linearAcceleration.x;
    break;

    case SH2_ARVR_STABILIZED_RV:
      QuaternionToEulerRV(&_sensorValue.un.arvrStabilizedRV, &_ypr, true);
    break;
  }

  // Check if calibration is done yet
  if (_calibrationFlag == 1)
  {
    for (uint16_t i = 0; i < _calibrationCount; i++) 
    {
      _sumPitch += _ypr._pitch;
      _sumRoll += _ypr._roll;
      _sumYaw += _ypr._yaw;
    }
  _imuCAL._pitchOffset = _sumPitch / _calibrationCount;
  _imuCAL._rollOffset = _sumRoll / _calibrationCount;
  
  _calibrationFlag = 0;
  }

}
