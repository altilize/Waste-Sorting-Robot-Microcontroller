// ========== FUNGSI HOLONOMIC =============================
void holonomic(float vx, float vy, float vz) {
  float holonomic_speedA = (-0.35 * vx) + (0.35 * vy) + (0.25 * vz);
  float holonomic_speedB = (-0.35 * vx) + (-0.35 * vy) + (0.25 * vz);
  float holonomic_speedC = (0.35 * vx) + (-0.35 * vy) + (0.25 * vz);
  float holonomic_speedD = (0.35 * vx) + (0.35 * vy) + (0.25 * vz);

  float maxRPM = 100;

  setpoint1 = holonomic_speedA * maxRPM;
  setpoint2 = holonomic_speedB * maxRPM;
  setpoint3 = holonomic_speedC * maxRPM;
  setpoint4 = holonomic_speedD * maxRPM;

  if (setpoint1 < 0) {
    setpoint1 *= -1;
    flagPID1 = true;
  } else if (setpoint1 > 0) {
    flagPID1 = false;
  } else if (setpoint1 == 0) {
    setpoint1 = 0;
  }

  if (setpoint2 < 0) {
    setpoint2 *= -1;
    flagPID2 = true;
  } else if (setpoint2 > 0) {
    flagPID2 = false;
  } else if (setpoint2 == 0) {
    setpoint2 = 0;
  }

  if (setpoint3 < 0) {
    setpoint3 *= -1;
    flagPID3 = true;
  } else if (setpoint3 > 0) {
    flagPID3 = false;
  } else if (setpoint3 == 0) {
    setpoint3 = 0;
  }

  if (setpoint4 < 0) {
    setpoint4 *= -1;
    flagPID4 = true;
  } else if (setpoint4 > 0) {
    flagPID4 = false;
  } else if (setpoint4 == 0) {
    setpoint4 = 0;
  }
  // flagPID1 = (setpoint1 < 0);
  // flagPID2 = (setpoint2 < 0);
  // flagPID3 = (setpoint3 < 0);
  // flagPID4 = (setpoint4 < 0);

  pid();
  // Serial.print(setpoint1);
  // Serial.print("  ");
  // Serial.print(setpoint2);
  // Serial.print("  ");
  // Serial.print(setpoint3);
  // Serial.print("  ");
  // Serial.println(setpoint4);
  Serial.print(SpeedA);
  Serial.print("  ");
  Serial.print(SpeedB);
  Serial.print("  ");
  Serial.print(SpeedC);
  Serial.print("  ");
  Serial.println(SpeedD);
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
  analogWriteFrequency(20000);  // biar motor ga bunyi

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
