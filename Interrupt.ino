void initial_setup() {
  Serial.begin(115200);
  // ----------- Inisiasi Motor -------------------
  pinMode(AmotorR, OUTPUT);
  pinMode(AmotorL, OUTPUT);
  pinMode(BmotorR, OUTPUT);
  pinMode(BmotorL, OUTPUT);
  pinMode(CmotorR, OUTPUT);
  pinMode(CmotorL, OUTPUT);
  pinMode(DmotorR, OUTPUT);
  pinMode(DmotorL, OUTPUT);
  pinMode(Enable, OUTPUT);

  // --------- Inisiasi Encoder Motor ------------
  pinMode(AmotorENC, INPUT);
  pinMode(BmotorENC, INPUT);
  pinMode(CmotorENC, INPUT);
  pinMode(DmotorENC, INPUT);

  pinMode(Encoder1A, INPUT_PULLUP);
  pinMode(Encoder1B, INPUT_PULLUP);
  pinMode(Encoder2A, INPUT_PULLUP);
  pinMode(Encoder2B, INPUT_PULLUP);
  pinMode(Encoder3A, INPUT_PULLUP);
  pinMode(Encoder3B, INPUT_PULLUP);

  // ---------- Interupt ----------
  attachInterrupt(digitalPinToInterrupt(AmotorENC), ISR_encoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(BmotorENC), ISR_encoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(CmotorENC), ISR_encoder3, RISING);
  attachInterrupt(digitalPinToInterrupt(DmotorENC), ISR_encoder4, RISING);

  attachInterrupt(digitalPinToInterrupt(Encoder1A), encA, RISING);
  attachInterrupt(digitalPinToInterrupt(Encoder2A), encB, RISING);
  attachInterrupt(digitalPinToInterrupt(Encoder3A), encC, RISING);

  analogWriteFrequency(20000);  // biar motor ga bunyi

  inisiasi_timer();
}
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

void encA() {
  (digitalRead(Encoder1A) == digitalRead(Encoder1B)) ? Odometry1-- : Odometry1++;
}
void encB() {
  (digitalRead(Encoder2A) == digitalRead(Encoder2B)) ? Odometry2++ : Odometry2--;
}
void encC() {
  (digitalRead(Encoder3A) == digitalRead(Encoder3B)) ? Odometry3++ : Odometry3--;
}
//===========================================================

void inisiasi_timer() {
// Kalau pake STM32F4 tidak perlu diubah
  #if defined(TIM5)
    TIM_TypeDef *Instance = TIM5;
  #else
    TIM_TypeDef *Instance = TIM5;
  #endif

  HardwareTimer *MyTim = new HardwareTimer(Instance);
  MyTim->setOverflow(10, HERTZ_FORMAT);  // 10 Hz | 1/t = 10hz, t = 100 ms
  MyTim->attachInterrupt(updateoverflow);
  MyTim->resume();
}

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