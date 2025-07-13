// ========== FUNGSI HOLONOMIC =============================
void holonomic(float vx, float vy, float vz) {
  float holonomic_speedA = (-0.35 * vx) + (0.35 * vy) + (0.25 * vz);
  float holonomic_speedB = (-0.35 * vx) + (-0.35 * vy) + (0.25 * vz);
  float holonomic_speedC = (0.35 * vx) + (-0.35 * vy) + (0.25 * vz);
  float holonomic_speedD = (0.35 * vx) + (0.35 * vy) + (0.25 * vz);

  float maxRPM = 150.0;
  setpoint1 = holonomic_speedA * maxRPM;
  setpoint2 = holonomic_speedB * maxRPM;
  setpoint3 = holonomic_speedC * maxRPM;
  setpoint4 = holonomic_speedD * maxRPM;

  flagPID1 = (setpoint1 < 0);
  flagPID2 = (setpoint2 < 0);
  flagPID3 = (setpoint3 < 0);
  flagPID4 = (setpoint4 < 0);

  pid();

  // -------- Mengendalikan Motor --------- //
  controlMotor(AmotorR, AmotorL, SpeedA);
  controlMotor(BmotorR, BmotorL, SpeedB);
  controlMotor(CmotorR, CmotorL, SpeedC);
  controlMotor(DmotorR, DmotorL, SpeedD);
}

// ========= FUNGSI GERAKIN MOTOR DARI HOLONOMIC ========== //
void controlMotor(int motorR, int motorL, float speed) {
  digitalWrite(Enable, HIGH);

  int pwmValue = constrain(abs(speed), 0, 255);

  if (speed > 0) {
    analogWrite(motorR, pwmValue);
    analogWrite(motorL, 0);
  } else if (speed < 0) {
    analogWrite(motorR, 0);
    analogWrite(motorL, pwmValue);
  } else {
    analogWrite(motorR, 0);
    analogWrite(motorL, 0);
  }
}
