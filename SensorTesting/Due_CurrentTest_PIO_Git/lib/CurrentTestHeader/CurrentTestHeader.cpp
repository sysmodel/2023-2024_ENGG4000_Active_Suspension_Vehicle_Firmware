#include "CurrentTestHeader.h"

//------------------------------------------------------------------

// Voltage sensor
static int sensVVal;
static int scaledVolt;
float voltage;

// Loadcell
long readingSG;
HX711 scale;  // object for strain gauge
const int LOADCELL_DOUT_PIN = 12;
const int LOADCELL_SCK_PIN = 13;

// PID
int pwmOffset = 0;
int pwm;
double setIPID, currentPID, outPID; // define PID variable
PID myPID(&currentPID, &outPID, &setIPID, Kp, Ki, Kd, DIRECT);

// Current sensor (INA260)
Adafruit_INA260 ina260 = Adafruit_INA260();
float currentINA;
float currentOffsetINA;
static float currentSum = 0;
static int offsetWindow = 1000;

//------------------------------------------------------------------

void InitStuff() {
  Serial.begin(115200);

  pinMode(sensV, INPUT);
  pinMode(mdEn, OUTPUT);
  pinMode(mdIn1, OUTPUT);
  pinMode(mdIn2, OUTPUT);
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);

  Wire.begin();

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);

  if (!ina260.begin()) {
    Serial.println("Couldn't find INA260 chip");
    while (1);
  }
  Serial.println("Found INA260 chip");
  ina260.setAveragingCount(INA260_COUNT_4);
  // ina260.setCurrentConversionTime(INA260_TIME_8_244_ms);

  // CalibrateCurrent();
}

void GetVoltage() {
  sensVVal = analogRead(sensV);
  scaledVolt = map(sensVVal, 0, 1023, 0, 3300);
  voltage = float(scaledVolt) * 0.005;  // in volts
}

void SetDirec(String dir) {
  if (dir == "CW") {
    digitalWrite(mdIn1, LOW);
    digitalWrite(mdIn2, HIGH);
  } else if (dir == "CCW") {
    digitalWrite(mdIn1, HIGH);
    digitalWrite(mdIn2, LOW);
  }
}

void GetCurrentINA(float offset) {
  currentINA = float(ina260.readCurrent())/1000.0;
  // currentINA = -0.015*currentINA*currentINA + 1.1545*currentINA + 0.0286;
  if (currentINA < 0) {
    currentINA = -currentINA;
  }
  currentINA = 1.0637*currentINA + 0.1416;
}

void CalibrateCurrentINA() {
  analogWrite(mdEn, 0);
  currentSum = 0;
  for (int j = 0; j < offsetWindow; j++) {
    GetCurrentINA(0.0);
    currentSum += float(currentINA);
    delay(5);
  }
  currentOffsetINA = currentSum / float(offsetWindow);
  Serial.print("Current offset: ");Serial.println(currentOffsetINA,3);
}