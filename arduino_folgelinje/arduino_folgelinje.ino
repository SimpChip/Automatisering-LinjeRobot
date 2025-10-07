#define AIN1 11
#define BIN1 10
#define AIN2 9
#define BIN2 6
#define PWMA 3
#define PWMB 5
#define STBY 2

#define emitter 12
#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 6;
uint8_t qtrPins[SensorCount] = {14, 15, 16, 17, 18, 19};

unsigned int sensorValues[SensorCount];


float error = 0;
float last_error = 0;

int lastSide = 1;

int sensorSum = 0;


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
    Serial.print("-");
  }
  Serial.println(); 


  // If sensor value is over 1400, we sett the value of the sensor to 1, if else, its not the line and we sett the value to 0
  int binValues[SensorCount];
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > 1400) {
      binValues[i] = 1;   // line
    } else {
      binValues[i] = 0;   // background
    }
  }

  //Weighted sum is based on all binary values index, and if we deivde the weighted sum with sum we get the posistion.
  long weightedSum = 0;
  long sum = 0;
  for (uint8_t i = 0; i < SensorCount; i++) {
    weightedSum += (long)binValues[i] * (i + 1);
    sum += binValues[i];
  }

  unsigned long lastSearchTime = 0;
  const int searchInterval = 50;
  
  //if sum is 0, the sensor cannot see any line, then it will turn based on the last error until it is back on the line
  if (sum == 0) {
      if (millis() - lastSearchTime > searchInterval) {
          lastSide = (last_error < 0) ? -1 : 1;
          int searchSpeed = map(abs(last_error), 0, 3.5, 120, 60);
          moveMotor(AIN1, AIN2, PWMA, searchSpeed * lastSide);
          moveMotor(BIN1, BIN2, PWMB, -searchSpeed * lastSide);
          lastSearchTime = millis();
      }
      return;
  } else {

    float pos = (float)weightedSum / sum; // [1..6]
    error = pos - 3.5;

    lastSide = (error < 0) ? -1 : 1;
    float delta_error = error - last_error; // derivative term
    last_error = error;
        
    // Constants
    const int MAX_SPEED = 140;
    const int MIN_SPEED = 80;
    const int PWM_MIN = 60;   // Consistent lower PWM limit everywhere

    // PID gains
    const float Kp = 30.0;
    const float Kd = 20.0;
    const float K = 6.0;

    float correction = Kp * error + Kd * delta_error;
    correction = constrain(correction, -120, 120);

    // Dynamic speed scaling
    float dynamic_factor = abs(error) + 0.8f * abs(delta_error);

    // Map dynamic factor to speed (reversed scale!)
    int base_speed = map(dynamic_factor, 0, K, MAX_SPEED-correction, PWM_MIN);
    base_speed = constrain(base_speed, MIN_SPEED, MAX_SPEED);
 
    // Apply correction
    int leftSpeed  = constrain(base_speed - correction, PWM_MIN, MAX_SPEED);
    int rightSpeed = constrain(base_speed + correction, PWM_MIN, MAX_SPEED);

    moveMotor(AIN1, AIN2, PWMA, -leftSpeed);
    moveMotor(BIN1, BIN2, PWMB, -rightSpeed);
  }
}

void moveMotor(int in1, int in2, int pwm, int speed) {
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwm, speed);
  } else if (speed < 0) {
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