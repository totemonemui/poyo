// モータの PWM を設定する関数
void setMotorPulse(int left, int right) {
  left = left * RATIO_RLSPEED;
  if(left > 0) {
    analogWrite(MOTOR_L_IN1, min(left, 255)); analogWrite(MOTOR_L_IN2, 0);
  } else {
    analogWrite(MOTOR_L_IN1, 0); analogWrite(MOTOR_L_IN2, min(-left, 255));
  }
  if(right > 0) {
    analogWrite(MOTOR_R_IN1, min(right, 255)); analogWrite(MOTOR_R_IN2, 0);
  } else {
    analogWrite(MOTOR_R_IN1, 0); analogWrite(MOTOR_R_IN2, min(-right, 255));
  }
}
