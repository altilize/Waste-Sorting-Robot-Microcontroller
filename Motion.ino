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

  flagPID1 = (setpoint1 < 0);
  flagPID2 = (setpoint2 < 0);
  flagPID3 = (setpoint3 < 0);
  flagPID4 = (setpoint4 < 0);

  setpoint1 = abs(setpoint1);
  setpoint2 = abs(setpoint2);
  setpoint3 = abs(setpoint3);
  setpoint4 = abs(setpoint4);

  pid();

  Serial.print(SpeedA);
  Serial.print("  ");
  Serial.print(SpeedB);
  Serial.print("  ");
  Serial.print(SpeedC);
  Serial.print("  ");
  Serial.println(SpeedD);
  
  motorauto();
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

void motorauto() {
  digitalWrite(Enable, HIGH);

  // -------- motor A ------------
  if (SpeedA > 0) {
    analogWrite(AmotorL, 0);
    analogWrite(AmotorR, SpeedA);
  } else if (SpeedA < 0) {
    SpeedA *= -1;
    analogWrite(AmotorL, SpeedA);
    analogWrite(BmotorR, 0);
  } else if (SpeedA == 0) {
    SpeedA = 0;
    analogWrite(AmotorL, 0);
    analogWrite(AmotorR, 0);
  }


  // -------- motor B ------------
  if (SpeedB > 0) {
    analogWrite(BmotorL, SpeedB);
    analogWrite(BmotorR, 0);
  }

  else if (SpeedB < 0) {
    SpeedB *= -1;
    analogWrite(BmotorL, 0);
    analogWrite(BmotorR, SpeedB);
  } else if (SpeedB == 0) {
    SpeedB = 0;
    analogWrite(BmotorL, 0);
    analogWrite(BmotorR, 0);
  }


  // -------- motor C ------------
  if (SpeedC > 0) {
    analogWrite(CmotorL, 0);
    analogWrite(CmotorR, SpeedC);
  }

  else if (SpeedC < 0) {
    SpeedC *= -1;
    analogWrite(CmotorL, SpeedC);
    analogWrite(CmotorR, 0);
  } else if (SpeedC == 0) {
    SpeedC = 0;
    analogWrite(CmotorL, 0);
    analogWrite(CmotorR, 0);
  }


  // -------- motor D ------------
  if (SpeedD > 0) {
    analogWrite(DmotorL, SpeedD);
    analogWrite(DmotorR, 0);
  }

  else if (SpeedD < 0) {
    SpeedD *= -1;
    analogWrite(DmotorL, 0);
    analogWrite(DmotorR, SpeedD);
  } else if (SpeedD == 0) {
    SpeedD = 0;
    analogWrite(DmotorL, 0);
    analogWrite(DmotorR, 0);
  }
}
