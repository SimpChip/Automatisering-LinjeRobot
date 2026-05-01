
void moveMotor(int in1, int in2, int ch, int speed) {
  speed = constrain(speed, -255, 255);
  if      (speed > 0) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  }
  else if (speed < 0) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
  else                { digitalWrite(in1, LOW);  digitalWrite(in2, LOW);  }
  ledcWrite(ch, abs(speed));
}

void stopMotors() {
  moveMotor(AIN1, AIN2, CHANNEL_A, 0);
  moveMotor(BIN1, BIN2, CHANNEL_B, 0);
}

void setMotors(int leftSpeed, int rightSpeed) {
  if (rightSpeed < 0) moveMotor(AIN1, AIN2, CHANNEL_A, rightSpeed - MOTOR_TRIM);
  else                moveMotor(AIN1, AIN2, CHANNEL_A, rightSpeed + MOTOR_TRIM);
  moveMotor(BIN1, BIN2, CHANNEL_B, leftSpeed);
}