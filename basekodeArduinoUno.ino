#define AIN1 16
#define AIN2 17

#define BIN1 15
#define BIN2 14

#define STBY 7

#define PWM_FREQ 20000
#define PWM_RES  8       
#define PWMA 10
#define PWMB 11

#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 11;
uint8_t qtrPins[SensorCount] = {2,3,4,5,6,18,19,8,9,12,13};

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

  pinMode(PWMA, OUTPUT);      
  pinMode(PWMB, OUTPUT);
  
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
  moveMotor(AIN1, AIN2, PWMA, 200);
  moveMotor(BIN1, BIN2, PWMB, 200); 
}



void moveMotor(int in1, int in2, int pwm, int speed) {
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwm, speed);   // UNO PWM
  } 
  else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwm, -speed);  // UNO PWM
  } 
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwm, 0);
  }
}

