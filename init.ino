void initial_setup() {
// inisiasi Serial dan I2C dan analogfreq ada di main

  init_Motor();
  init_Encoder();
  init_Interrupt();
  init_Lidar();
  init_Timer();
}

void init_Timer() {
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

void init_Lidar() {
  for (uint8_t i = 0; i < 2; i++) { // for loop dari 0 sampai jumlah lidar (4)
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }
  for (uint8_t i = 0; i < 2; i++) {
    pinMode(xshutPins[i], INPUT);
    delay(10);
    lidar[i].setTimeout(500);
    if (!lidar[i].init()) {
      Serial.print("Gagal Membaca Sensor ");
      Serial.println(i);
    }
    lidar[i].setAddress(0x2A + i);
    lidar[i].startContinuous(50);

  }
}

void init_Encoder() {
  // --------- Inisiasi Encoder Motor ------------
  pinMode(AmotorENC, INPUT);
  pinMode(BmotorENC, INPUT);
  pinMode(CmotorENC, INPUT);
  pinMode(DmotorENC, INPUT);

  // ------- Inisiasi Encoder Rotary -------------
  pinMode(Encoder1A, INPUT_PULLUP);
  pinMode(Encoder1B, INPUT_PULLUP);
  pinMode(Encoder2A, INPUT_PULLUP);
  pinMode(Encoder2B, INPUT_PULLUP);
  pinMode(Encoder3A, INPUT_PULLUP);
  pinMode(Encoder3B, INPUT_PULLUP);
}

void init_Motor() {
  pinMode(AmotorR, OUTPUT);
  pinMode(AmotorL, OUTPUT);
  pinMode(BmotorR, OUTPUT);
  pinMode(BmotorL, OUTPUT);
  pinMode(CmotorR, OUTPUT);
  pinMode(CmotorL, OUTPUT);
  pinMode(DmotorR, OUTPUT);
  pinMode(DmotorL, OUTPUT);
  pinMode(Enable, OUTPUT);
}

void init_Interrupt() {
  attachInterrupt(digitalPinToInterrupt(AmotorENC), ISR_encoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(BmotorENC), ISR_encoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(CmotorENC), ISR_encoder3, RISING);
  attachInterrupt(digitalPinToInterrupt(DmotorENC), ISR_encoder4, RISING);

  attachInterrupt(digitalPinToInterrupt(Encoder1A), encA, RISING);
  attachInterrupt(digitalPinToInterrupt(Encoder2A), encB, RISING);
  attachInterrupt(digitalPinToInterrupt(Encoder3A), encC, RISING);
}