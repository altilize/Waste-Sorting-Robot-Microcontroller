void initial_setup() {
  // inisiasi Serial dan I2C dan analogfreq ada di main

  init_Motor();
  init_Encoder();
  init_Suction();
  init_Interrupt();
  init_Lidar();
  init_Timer();
  init_Compass();
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
  for (uint8_t i = 0; i < 2; i++) {  // for loop dari 0 sampai jumlah lidar (4)
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
  pinMode(AmotorENC_PIN, INPUT_PULLUP);
  pinMode(BmotorENC_PIN, INPUT_PULLUP);
  pinMode(CmotorENC_PIN, INPUT_PULLUP);
  pinMode(DmotorENC_PIN, INPUT_PULLUP);

  // ------- Inisiasi Encoder Rotary -------------
  pinMode(ODOMETRY_PIN_1A, INPUT_PULLUP);
  pinMode(ODOMETRY_PIN_1B, INPUT_PULLUP);
  pinMode(ODOMETRY_PIN_2A, INPUT_PULLUP);
  pinMode(ODOMETRY_PIN_2B, INPUT_PULLUP);
  pinMode(ODOMETRY_PIN_3A, INPUT_PULLUP);
  pinMode(ODOMETRY_PIN_3B, INPUT_PULLUP);

  // ------- Inisiasi Encoder Arm ----------------
  pinMode(ARM_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ARM_ENCODER_B_PIN, INPUT_PULLUP);
}

void init_Suction() {
  pinMode(SUCTION_PIN, OUTPUT);
  digitalWrite(SUCTION_PIN, LOW);
}

void init_Motor() {
  pinMode(AmotorR_PIN, OUTPUT);
  pinMode(AmotorL_PIN, OUTPUT);
  pinMode(BmotorR_PIN, OUTPUT);
  pinMode(BmotorL_PIN, OUTPUT);
  pinMode(CmotorR_PIN, OUTPUT);
  pinMode(CmotorL_PIN, OUTPUT);
  pinMode(DmotorR_PIN, OUTPUT);
  pinMode(DmotorL_PIN, OUTPUT);
  pinMode(ENABLE_MOTOR_PIN, OUTPUT);

  pinMode(ARM_FORWARD_PIN, OUTPUT);
  pinMode(ARM_BACKWARD_PIN, OUTPUT);
  pinMode(ARM_LIMIT, INPUT);
}

void init_Interrupt() {
  attachInterrupt(digitalPinToInterrupt(AmotorENC_PIN), ISR_encoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(BmotorENC_PIN), ISR_encoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(CmotorENC_PIN), ISR_encoder3, RISING);
  attachInterrupt(digitalPinToInterrupt(DmotorENC_PIN), ISR_encoder4, RISING);

  attachInterrupt(digitalPinToInterrupt(ODOMETRY_PIN_1A), encA, RISING);
  attachInterrupt(digitalPinToInterrupt(ODOMETRY_PIN_2A), encB, RISING);
  attachInterrupt(digitalPinToInterrupt(ODOMETRY_PIN_3A), encC, RISING);

  attachInterrupt(digitalPinToInterrupt(ARM_ENCODER_A_PIN), ISR_encoderARM, RISING);
}

void init_Compass() {
  if (!bno.begin()) {
    Serial.print("[i] Kompasnya ga kebaca elektrik!");
    while (1)
      ; 
  }

  delay(1000);  // kalau ganggu nanti hapus aja
}