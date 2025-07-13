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
  pinMode(AmotorENC, INPUT_PULLUP);
  pinMode(BmotorENC, INPUT_PULLUP);
  pinMode(CmotorENC, INPUT_PULLUP);
  pinMode(DmotorENC, INPUT_PULLUP);

  pinMode(PD14, OUTPUT);  // LED BUILTIN, buat debug Encoder motor

  // ---------- Interupt ----------
  attachInterrupt(digitalPinToInterrupt(AmotorENC), ISR_encoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(BmotorENC), ISR_encoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(CmotorENC), ISR_encoder3, RISING);
  attachInterrupt(digitalPinToInterrupt(DmotorENC), ISR_encoder4, RISING);

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
//===========================================================

void inisiasi_timer() {
  // Kalau pake STM32F4 tidak perlu diubah
  #if defined(TIM5)
    TIM_TypeDef *Instance = TIM5;
  #else
    TIM_TypeDef *Instance = TIM5;
  #endif

  HardwareTimer *MyTim = new HardwareTimer(Instance);
  MyTim->setOverflow(10, HERTZ_FORMAT); // 10 Hz (100 ms)
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

  rpmA = (encoder1RPM * 60 * 10) / 134;  // Encoder * 60 detik/ 0.1 detik (interveal 100ms) * (ppr * rasio motor)
  rpmB = (encoder2RPM * 60 * 10) / 134;
  rpmC = (encoder3RPM * 60 * 10) / 134;
  rpmD = (encoder4RPM * 60 * 10) / 134;
  /*
  Serial.print(rpmA);
  Serial.print("|");
  Serial.print(rpmB);
  Serial.print("|");
  Serial.print(rpmC);
  Serial.println();
  */
}