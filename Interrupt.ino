// ============= Interupt Encoder Motor =====================
void ISR_encoder1() {
  encoderMotor1++;
}
void ISR_encoder2() {
  encoderMotor2++;
}
void ISR_encoder3() {
  encoderMotor3++;
}
void ISR_encoder4() {
  encoderMotor4++;
}

// ============== Interrupt Rotary Encoder ==================
void encA() {
  (digitalRead(Encoder1A) == digitalRead(Encoder1B)) ? Odometry1-- : Odometry1++;
}
void encB() {
  (digitalRead(Encoder2A) == digitalRead(Encoder2B)) ? Odometry2++ : Odometry2--;
}
void encC() {
  (digitalRead(Encoder3A) == digitalRead(Encoder3B)) ? Odometry3++ : Odometry3--;
}

// ============= Interrupt Timer  ============================
void updateoverflow() {
  encoder1RPM = encoderMotor1;
  encoder2RPM = encoderMotor2;
  encoder3RPM = encoderMotor3;
  encoder4RPM = encoderMotor4;

  encoderMotor1 = 0;
  encoderMotor2 = 0;
  encoderMotor3 = 0;
  encoderMotor4 = 0;

  rpmA = (encoder1RPM * 60 * 10) / 134;  // Encoder * 60 detik/ 0.1 detik (interval 100ms) * (ppr * rasio motor)
  rpmB = (encoder2RPM * 60 * 10) / 134;
  rpmC = (encoder3RPM * 60 * 10) / 134;
  rpmD = (encoder4RPM * 60 * 10) / 134;

  // pid(); kalau mau PIDnya setiap 10 Hz, harus di bagi interval di function PIDnya

  // Serial.print(rpmA);
  // Serial.print("|");
  // Serial.print(rpmB);
  // Serial.print("|");
  // Serial.print(rpmC);
  // Serial.print("|");
  // Serial.print(rpmD);
  // Serial.println();
}


