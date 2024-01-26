#include "TimerOne.h"
#include "Encoder.h"

Encoder encoder(2,3);

int32_t lastCount = 0;
int32_t count = 0;
unsigned long lastTime = micros();
unsigned long time = 0;
double encoderVel = 0;
int ppr = 2048;

void setup() 
{
  Serial.begin(115200);

  Timer1.initialize(10000);
  Timer1.attachInterrupt(GetEncoderSpeed);

}

void loop() 
{

  Serial.print("Pulse Count: ");
  Serial.print(encoder.read());
  Serial.print("\t");
  Serial.print("Velocity (rad/s): ");
  Serial.println(encoderVel);

  delay(500);

}

void GetEncoderSpeed()
{
  count = encoder.read();
  time = micros();

  encoderVel = (double(count - lastCount) * 2 * PI) / (double((time - lastTime) * ppr * 1e-6)); 

  lastCount = count;
  lastTime = time;
}