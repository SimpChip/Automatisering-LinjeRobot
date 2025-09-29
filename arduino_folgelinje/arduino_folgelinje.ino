
// Motor Driver Pins
#define AIN1 11
#define BIN1 10 
#define AIN2 9
#define BIN2 6
#define PWMA 3 // PMW motor A
#define PWMB 5 // PMW motor B
#define STBY 2

// Pin for emitter
#define emitter 12

// Zumo reflection array libary
#include <QTRSensors.h>

QTRSensors qtr;

//Amount of senors
const uint8_t SensorCount = 6;
//Sensor pins is anlog pins A0-A5 turned to digital pins
uint8_t qtrPins[SensorCount] = {14, 15, 16, 17, 18, 19};

unsigned int sensorValues[SensorCount];

int max_speed = 150;  
int min_speed = 60; 

float error = 0;
float last_error = 0;

int lastSide = 1;


void setup() {

  Serial.begin(115200);

  qtr.setTypeRC();
  qtr.setSensorPins(qtrPins, SensorCount);

  qtr.setEmitterPin(emitter);


  delay(1000);
  Serial.println("Calibrating...");

  // Calibrate
  for (uint16_t i = 0; i < 250; i++) {
    qtr.calibrate();
    delay(20);
  }

  Serial.println("Calibration complete.");

  // Set pins as outputs
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  
  pinMode(STBY, OUTPUT);

  // Take the motor driver out of standby
  digitalWrite(STBY, HIGH);
 

}

void loop() {

  qtr.read(sensorValues);

  // Print sensor readings
  for (uint8_t i = 0; i < 6; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();

  //Turning Sensor values between 0-2500 into binary values 1 if over 1750 and 0 if lower
  int binValues[SensorCount];
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > 1750) {
      binValues[i] = 1;   // line
    } else {
      binValues[i] = 0;   // background
    }
  }

  long weightedSum = 0;
  long sum = 0;
  for (uint8_t i = 0; i < SensorCount; i++) {
    weightedSum += (long)binValues[i] * (i + 1);
    sum += binValues[i];
  }

  unsigned long lastSearchTime = 0;
  const int searchInterval = 50;
  
  //if sum is 0, sensor is out of route, Using last error to continue last correction 
  if (sum == 0) {
      if (millis() - lastSearchTime > searchInterval) {
          lastSide = (last_error < 0) ? -1 : 1;
          int searchSpeed = map(abs(last_error), 0, 3.5, 90, 50);
          moveLeftMotor( searchSpeed * lastSide);
          moveRightMotor( -searchSpeed * lastSide);
          lastSearchTime = millis();
      }
      return;
  }

  float pos = (float)weightedSum / sum;   // [1..6]
  error = pos - 3.5;

  lastSide = (error < 0) ? -1 : 1;
  float delta_error = error - last_error;
  last_error = error;

  float Kp = 30.0;
  float Kd = 10.0;      
  float k  = 10.0;

  float correction = Kp * error + Kd * delta_error;
  correction = constrain(correction, -120, 120);

  float dynamic_factor = abs(error) + 0.8 * abs(delta_error);

  int base_speed = map(dynamic_factor, 0, k, max_speed, 60); 
  base_speed = constrain(base_speed, min_speed, max_speed);


  int leftSpeed  = base_speed - correction;
  int rightSpeed = base_speed + correction;

  // Limit speed range
  leftSpeed  = constrain(leftSpeed, 50, 200);
  rightSpeed = constrain(rightSpeed, 50, 200);


  Serial.println(leftSpeed);
  Serial.println(rightSpeed);

  //moving both motors based on speed calculated
  moveLeftMotor(leftSpeed);
  moveRightMotor(rightSpeed);
  

}

void stop(int stopTime) {
  moveMotor(AIN1, AIN2, PWMA, 0);
  moveMotor(BIN1, BIN2, PWMB, 0);
  delay(stopTime);
}


void moveMotor(int in1, int in2, int pwm, int speed) {
  if (speed < 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwm, speed);
  } else if (speed > 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwm, -speed);
  } else {
    // Stop motor
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwm, 0);
  }
}

void moveRightMotor(int speed) {
  moveMotor(BIN1, BIN2, PWMB, speed);
}

void moveLeftMotor(int speed) {
  moveMotor(AIN1, AIN2, PWMA, speed);
}