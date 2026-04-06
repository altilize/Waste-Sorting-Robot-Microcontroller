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
  const int min_error = 5;     // Toleransi error encoder
  const float Kp = 0.8;        // Konstanta Proporsional
  
  // Hitung Error
  int error = arm_target_position - encoderarm_count;

  // Deadband: Jika error sangat kecil, matikan motor (supaya tidak berdengung/oscillate)
  if (abs(error) < min_error) {
    analogWrite(ARM_FORWARD_PIN, 0);
    analogWrite(ARM_BACKWARD_PIN, 0);
    return; // Keluar fungsi, hemat komputasi
  }

  // Hitung PWM Proporsional
  int pwm = Kp * error;

  // Batasi PWM (Clamping)
  pwm = constrain(pwm, -255, 255);
  
  // Tambahan: Minimum Power (agar motor tidak stuck saat PWM kecil tapi belum sampai)
  // Misal motor butuh minimal PWM 40 untuk mulai bergerak
  if (pwm > 0 && pwm < 40) pwm = 40;
  if (pwm < 0 && pwm > -40) pwm = -40;

  // Eksekusi ke Motor Driver
  if (pwm > 0) {
    analogWrite(ARM_FORWARD_PIN, pwm);
    analogWrite(ARM_BACKWARD_PIN, 0);
  } else {
    analogWrite(ARM_FORWARD_PIN, 0);
    analogWrite(ARM_BACKWARD_PIN, abs(pwm));
  }
}



// ======================= Leadscrew Function ================== //
void set_leadscrew_motor(int speed) {
  // Speed range: -255 (Full Down) sampai 255 (Full Up)
  if (speed > 0) {
    // Gerak Naik (Sesuaikan logika HIGH/LOW jika terbalik)
    analogWrite(LS_PIN_A, speed);
    analogWrite(LS_PIN_B, 0);
  } else if (speed < 0) {
    // Gerak Turun
    analogWrite(LS_PIN_A, 0);
    analogWrite(LS_PIN_B, -speed); // Jadikan positif untuk PWM
  } else {
    // Stop / Rem
    analogWrite(LS_PIN_A, 255); // High-High = Brake (tergantung driver)
    analogWrite(LS_PIN_B, 255); // Atau gunakan 0, 0 untuk coasting
  }
}

void update_leadscrew() {
  if (!ls_active) return; // Jangan lakukan apa-apa jika tidak diperintahkan

  long error = ls_target_pos - ls_current_pos;

  // Jika sudah dalam batas toleransi, berhenti
  if (abs(error) <= ls_tolerance) {
    set_leadscrew_motor(0);
    ls_active = false;
    Serial.println("Leadscrew: Target Reached");
    return;
  }

  // Kontrol Proporsional (P-Controller)
  // Semakin jauh target, semakin cepat. Semakin dekat, melambat.
  int speed = error * kp_leadscrew;

  // Batasi kecepatan (Clamping) agar tidak melebihi kemampuan PWM & Motor
  if (speed > 255) speed = 255;
  if (speed < -255) speed = -255;
  
  // Pastikan ada tenaga minimum agar motor tidak berdengung (Deadzone motor)
  if (speed > 0 && speed < 60) speed = 60;
  if (speed < 0 && speed > -60) speed = -60;

  set_leadscrew_motor(speed);
}
