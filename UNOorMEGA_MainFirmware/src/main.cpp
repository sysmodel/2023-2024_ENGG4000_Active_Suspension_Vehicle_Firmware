#include <Arduino.h>
#include "TimerOne.h"
#include "Encoder.h"
#include "Quad_Encoder_M.h"
#include "IR_Sensor_M.h"
#include "Abs_Encoders_M.h"

void setup()
{

  Serial.begin(115200);

  if (!VCNL4040.begin()) 
    {
      Serial.println("VCNL4040 not found!");
      while (1);
    }

  Serial.println("All Sensors on I2C Found!");

  InitializeVCNL4040();
}

void loop() 
{
  uint16_t proximityValue = GetProximity();
  double  quadEncoderSpeed = GetQuadEncoderSpeed();

  Serial.print("Proximity: ");
  Serial.print(proximityValue);
  Serial.print("  ");
  Serial.print("QuadEncoderSpeed: ");
  Serial.println(quadEncoderSpeed);


  delay(500);
}