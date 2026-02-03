#define AIN1 6
#define AIN2 7
#define BIN1 5
#define BIN2 4

#define STBY 8

#define PWM_FREQ 20000
#define PWM_RES  8       
#define PWMA 2
#define PWMB 3

#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 12;
uint8_t qtrPins[SensorCount] = {9,10,24, 23, 22, 21, 20, 19, 18, 17,11,12};

uint16_t sensorValues[SensorCount];
uint16_t whiteRefValues[SensorCount];


float error = 0;
float last_error = 0;

int lastSide = 1;

int sensorSum = 0;




void setup() {
  
  Serial.begin(115200);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);

  // Take the motor driver out of standby

  ledcSetup(PWMA,  PWM_FREQ, PWM_RES);
  ledcSetup(PWMB, PWM_FREQ, PWM_RES);

  ledcAttachPin(PWMA, PWMA);
  ledcAttachPin(PWMB, PWMB);

  qtr.setTypeRC();
  qtr.setSensorPins(qtrPins, SensorCount);

  delay(3000);
  Serial.println("Calibrating...");

  for (uint8_t i = 0; i < SensorCount; i++) {
    whiteRefValues[i] = 2500;
  }

  // Calibrate
  for (uint16_t i = 0; i < 200; i++) {
    qtr.calibrate();
    if (i < 50) {
      moveMotor(AIN1, AIN2, PWMA, 60);
      moveMotor(BIN1, BIN2, PWMB, -60);
    } else if (i < 150) {
      moveMotor(AIN1, AIN2, PWMA, -60);
      moveMotor(BIN1, BIN2, PWMB, 60);
    } else {
      moveMotor(AIN1, AIN2, PWMA, 60);
      moveMotor(BIN1, BIN2, PWMB, -60);
    }
    qtr.read(sensorValues);
    for (uint8_t i = 0; i < SensorCount; i++) {
      if (sensorValues[i] <= whiteRefValues[i] ) {
        whiteRefValues[i] = sensorValues[i];
      }
    }
  }



  Serial.println("Calibration complete.");
 
}

void loop() {
  qtr.read(sensorValues);

  // Print sensor readings with white noise offset
  for (uint8_t i = 0; i < SensorCount; i++) {
  Serial.print(sensorValues[i]-whiteRefValues[i]);
  Serial.print(" - ");
  }
  Serial.println();

  delay(50);

  //lager en array med 1 som representerer linje og 0 ingenting
  int changeTol = 50;
  int binValues[SensorCount];
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (sensorValues[i] - whiteRefValues[i] > changeTol) {
      binValues[i] = 1;   // line
      Serial.print(1);
    } else {
      binValues[i] = 0;   // backgroun
      Serial.print(0);
    }
    Serial.print(" - ");
  }
  Serial.println();

  long weightedSum = 0;
  long sum = 0;

  for (uint8_t i = 0; i < SensorCount; i++) {
    weightedSum += (long)binValues[i] * (i + 1); 
    sum += binValues[i]; 
  }

  // posisjon mellom 0 .. 12
  float pos = (float)weightedSum / sum;  
  
  int rightSpeed = 0;
  int leftSpeed = 0;
  moveMotor(AIN1, AIN2, PWMA, rightSpeed);
  moveMotor(BIN1, BIN2, PWMB, leftSpeed); 
}



void moveMotor(int in1, int in2, int pwm, int speed) {
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(pwm, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(pwm, -speed);
  } else {
    // Stop motor
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(pwm, 0);
  }
}
