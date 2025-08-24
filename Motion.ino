// ======================= Holonomic Function ================ // 
void holonomic(float vx, float vy, float vz) {
  float holonomic_speedA = (-0.35 * vx) + (0.35 * vy) + (0.25 * vz);
  float holonomic_speedB = (-0.35 * vx) + (-0.35 * vy) + (0.25 * vz);
  float holonomic_speedC = (0.35 * vx) + (-0.35 * vy) + (0.25 * vz);
  float holonomic_speedD = (0.35 * vx) + (0.35 * vy) + (0.25 * vz);

  setpoint1 = holonomic_speedA;
  setpoint2 = holonomic_speedB;
  setpoint3 = holonomic_speedC;
  setpoint4 = holonomic_speedD;

  flagPID1 = (setpoint1 < 0);
  flagPID2 = (setpoint2 < 0);
  flagPID3 = (setpoint3 < 0);
  flagPID4 = (setpoint4 < 0);

  setpoint1 = abs(setpoint1);
  setpoint2 = abs(setpoint2);
  setpoint3 = abs(setpoint3);
  setpoint4 = abs(setpoint4);

  pid();
  
  motorauto();
}

void motorauto() {
  digitalWrite(ENABLE_MOTOR_PIN, HIGH);

  // -------- motor A ------------
  if (SpeedA > 0) {
    analogWrite(AmotorL_PIN, 0);
    analogWrite(AmotorR_PIN, SpeedA);
  } else if (SpeedA < 0) {
    SpeedA *= -1;
    analogWrite(AmotorL_PIN, SpeedA);
    analogWrite(BmotorR_PIN, 0);
  } else if (SpeedA == 0) {
    SpeedA = 0;
    analogWrite(AmotorL_PIN, 0);
    analogWrite(AmotorR_PIN, 0);
  }


  // -------- motor B ------------
  if (SpeedB > 0) {
    analogWrite(BmotorL_PIN, SpeedB);
    analogWrite(BmotorR_PIN, 0);
  }

  else if (SpeedB < 0) {
    SpeedB *= -1;
    analogWrite(BmotorL_PIN, 0);
    analogWrite(BmotorR_PIN, SpeedB);
  } else if (SpeedB == 0) {
    SpeedB = 0;
    analogWrite(BmotorL_PIN, 0);
    analogWrite(BmotorR_PIN, 0);
  }


  // -------- motor C ------------
  if (SpeedC > 0) {
    analogWrite(CmotorL_PIN, 0);
    analogWrite(CmotorR_PIN, SpeedC);
  }

  else if (SpeedC < 0) {
    SpeedC *= -1;
    analogWrite(CmotorL_PIN, SpeedC);
    analogWrite(CmotorR_PIN, 0);
  } else if (SpeedC == 0) {
    SpeedC = 0;
    analogWrite(CmotorL_PIN, 0);
    analogWrite(CmotorR_PIN, 0);
  }


  // -------- motor D ------------
  if (SpeedD > 0) {
    analogWrite(DmotorL_PIN, SpeedD);
    analogWrite(DmotorR_PIN, 0);
  }

  else if (SpeedD < 0) {
    SpeedD *= -1;
    analogWrite(DmotorL_PIN, 0);
    analogWrite(DmotorR_PIN, SpeedD);
  } else if (SpeedD == 0) {
    SpeedD = 0;
    analogWrite(DmotorL_PIN, 0);
    analogWrite(DmotorR_PIN, 0);
  }
}

// ======================= Arm Function ====================== //
void controlArm() {
    static bool holding = false;
    static int last_target = 0;
    const int min_error = 5;
    const float Kp = 0.8; // Pake proportional ajaa

    if (arm_target_position != last_target) {
        holding = true;
        last_target = arm_target_position;
    }

    if (holding) {
        int error = arm_target_position - encoderarm_count;

        // Cek jika sudah sampai di target
        if (abs(error) < min_error) {
            analogWrite(ARM_FORWARD_PIN, 0);
            analogWrite(ARM_BACKWARD_PIN, 0);
            holding = false; // berhenti
            return;
        }
        // Menghitung PWM
        int pwm = Kp * error;
        pwm = constrain(pwm, -255, 255);
        analogWrite(ARM_FORWARD_PIN, pwm > 0 ? pwm : 0);
        analogWrite(ARM_BACKWARD_PIN, pwm < 0 ? -pwm : 0);

    } else {
        // motor mati jika tidak dalam mode holding
        analogWrite(ARM_FORWARD_PIN, 0);
        analogWrite(ARM_BACKWARD_PIN, 0);
    }
}
