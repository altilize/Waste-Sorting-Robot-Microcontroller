// ======================= Holonomic Function ================ //
void holonomic(float vx, float vy, float vz) {



  float holonomic_speedA = (-0.35 * vx) + (0.35 * vy) + (0.25 * vz);
  float holonomic_speedB = (-0.35 * vx) + (-0.35 * vy) + (0.25 * vz);
  float holonomic_speedC = (0.35 * vx) + (-0.35 * vy) + (0.25 * vz);
  float holonomic_speedD = (0.35 * vx) + (0.35 * vy) + (0.25 * vz);

  setpoint1 = holonomic_speedA * 10;
  setpoint2 = holonomic_speedB * 10;
  setpoint3 = holonomic_speedC * 10;
  setpoint4 = holonomic_speedD * 10;

  // SpeedA = holonomic_speedA;
  // SpeedB = holonomic_speedB;
  // SpeedC = holonomic_speedC;
  // SpeedD = holonomic_speedD;


  flagPID1 = (setpoint1 < 0);
  flagPID2 = (setpoint2 < 0);
  flagPID3 = (setpoint3 < 0);
  flagPID4 = (setpoint4 < 0);

  setpoint1 = abs(setpoint1);
  setpoint2 = abs(setpoint2);
  setpoint3 = abs(setpoint3);
  setpoint4 = abs(setpoint4);

  // pid ada di updateoverflow(); 10hz
  // pid();

  motorauto();
}

void motorauto() {
  digitalWrite(ENABLE_MOTOR_PIN, HIGH);

  int pwmA = SpeedA;
  int pwmB = SpeedB;
  int pwmC = SpeedC;
  int pwmD = SpeedD;

  // -------- motor A ------------
  if (pwmA > 0) {
    analogWrite(AmotorL_PIN, pwmA);
    analogWrite(AmotorR_PIN, 0);
  } else {
    analogWrite(AmotorL_PIN, 0);
    analogWrite(AmotorR_PIN, abs(pwmA));
  }

  // -------- motor B ------------
  if (pwmB > 0) {
    analogWrite(BmotorL_PIN, pwmB);
    analogWrite(BmotorR_PIN, 0);
  } else {
    analogWrite(BmotorL_PIN, 0);
    analogWrite(BmotorR_PIN, abs(pwmB));
  }

  // -------- motor C ------------
  if (pwmC > 0) {
    analogWrite(CmotorL_PIN, pwmC);
    analogWrite(CmotorR_PIN, 0);
  } else {
    analogWrite(CmotorL_PIN, 0);
    analogWrite(CmotorR_PIN, abs(pwmC));
  }

  // -------- motor D ------------
  if (pwmD > 0) {
    analogWrite(DmotorL_PIN, pwmD);  
    analogWrite(DmotorR_PIN, 0);
  } else {
    analogWrite(DmotorL_PIN, 0);
    analogWrite(DmotorR_PIN, abs(pwmD));
  }
}

// ======================= Arm Function ====================== //
void controlArm() {
  static bool holding = false;
  static int last_target = 0;
  const int min_error = 5;
  const float Kp = 0.8;  // Pake proportional ajaa

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
      holding = false;  // berhenti
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
