// ======================== LINE FOLLOWER ======================
void LF_Read() {
  MUX_Select(0);
  delayMicroseconds(100);
  LF_Vertikal[0] = analogRead(MUX_INPUT);

  MUX_Select(1);
  delayMicroseconds(100);
  LF_Vertikal[1] = analogRead(MUX_INPUT);

  MUX_Select(2);
  delayMicroseconds(100);
  LF_Vertikal[2] = analogRead(MUX_INPUT);

  MUX_Select(3);
  delayMicroseconds(100);
  LF_Vertikal[3] = analogRead(MUX_INPUT);

  MUX_Select(4);
  delayMicroseconds(100);
  LF_Vertikal[4] = analogRead(MUX_INPUT);

  MUX_Select(5);
  delayMicroseconds(100);
  LF_Vertikal[5] = analogRead(MUX_INPUT);

  MUX_Select(6);
  delayMicroseconds(100);
  LF_Vertikal[6] = analogRead(MUX_INPUT);

  MUX_Select(7);
  delayMicroseconds(100);
  LF_Vertikal[7] = analogRead(MUX_INPUT);

  MUX_Select(8);
  delayMicroseconds(100);
  LF_Vertikal[8] = analogRead(MUX_INPUT);

  MUX_Select(9);
  delayMicroseconds(100);
  LF_Vertikal[9] = analogRead(MUX_INPUT);

  MUX_Select(10);
  delayMicroseconds(100);
  LF_Vertikal[10] = analogRead(MUX_INPUT);

  MUX_Select(11);
  delayMicroseconds(100);
  LF_Vertikal[11] = analogRead(MUX_INPUT);

  MUX_Select(12);
  delayMicroseconds(100);
  LF_Vertikal[12] = analogRead(MUX_INPUT);

  MUX_Select(13);
  delayMicroseconds(100);
  LF_Vertikal[13] = analogRead(MUX_INPUT);

  MUX_Select(14);
  delayMicroseconds(100);
  LF_Vertikal[14] = analogRead(MUX_INPUT);
}

float LF_WeightedAverage() {
  float TotalWeight = 0;
  int TotalActiveSensor = 0;
  LF_Read();
  // reset values
  for (int i = 0; i <= 6; i++) {
    LF_Vertikal_Dig[i] = 0;
  }

  if (LF_Vertikal[0] >= 650) {
    TotalWeight += weights[0];
    TotalActiveSensor++;
    LF_Vertikal_Dig[0] = 1;
  }

  if (LF_Vertikal[1] >= 650) {
    TotalWeight += weights[1];
    TotalActiveSensor++;
    LF_Vertikal_Dig[1] = 1;
  }

  if (LF_Vertikal[2] >= 650) {
    TotalWeight += weights[2];
    TotalActiveSensor++;
    LF_Vertikal_Dig[2] = 1;
  }

  if (LF_Vertikal[3] >= 650) {
    TotalWeight += weights[3];
    TotalActiveSensor++;
    LF_Vertikal_Dig[3] = 1;
  }
  if (LF_Vertikal[4] >= 650) {
    TotalWeight += weights[4];
    TotalActiveSensor++;
    LF_Vertikal_Dig[4] = 1;
  }
  if (LF_Vertikal[5] >= 650) {
    TotalWeight += weights[5];
    TotalActiveSensor++;
    LF_Vertikal_Dig[5] = 1;
  }
  if (LF_Vertikal[6] >= 650) {
    TotalWeight += weights[6];
    TotalActiveSensor++;
    LF_Vertikal_Dig[6] = 1;
  }

  // Jika tidak ada garis
  if (TotalActiveSensor == 0) {
    return 0;
  }

  return TotalWeight / TotalActiveSensor;
}


//  ============== Debugging Line Follower ================ //
void Debug_LF_Vertikal() {
  float WA = LF_WeightedAverage();

  for (int i = 0; i < 15; i++) {
    if (LF_Vertikal[i] >= 400) {
      LF_Vertikal_Dig[i] = 1;
    } else {
      LF_Vertikal_Dig[i] = 0;
    }
    Serial.print(LF_Vertikal_Dig[i]);
    Serial.print("  ");
  }
  Serial.println();
  // Serial.print(LF_Vertikal[0]);
  // Serial.print(" ");
  // Serial.print(LF_Vertikal[1]);
  // Serial.print(" ");
  // Serial.print(LF_Vertikal[2]);
  // Serial.print(" ");
  // Serial.print(LF_Vertikal[3]);
  // Serial.print(" ");
  // Serial.print(LF_Vertikal[4]);
  // Serial.print(" ");
  // Serial.print(LF_Vertikal[5]);
  // Serial.print(" ");
  // Serial.print(LF_Vertikal[6]);
  // Serial.print(" ");
  // Serial.print(LF_Vertikal[7]);
  // Serial.print(" ");
  // Serial.print(LF_Vertikal[8]);
  // Serial.print(" ");
  // Serial.print(LF_Vertikal[9]);
  // Serial.print(" ");
  // Serial.print(LF_Vertikal[10]);
  // Serial.print(" ");
  // Serial.print(LF_Vertikal[11]);
  // Serial.print(" ");
  // Serial.print(LF_Vertikal[12]);
  // Serial.print(" ");
  // Serial.print(LF_Vertikal[13]);
  // Serial.print(" ");
  // Serial.print(LF_Vertikal[14]);


  // Serial.print("  | Output : ");
  // Serial.println(WA);
}

// ---------------------- Utama Algoritma ------------------ //
void HomeToConveyor() {
  // --- Konstanta untuk Kontrol Lidar ---
  const int TARGET_JARAK = 140;      // Target jarak ke halangan (cm)
  const int TOLERANSI_SEIMBANG = 3;  // Toleransi beda jarak kiri-kanan (cm)
  const int TOLERANSI_JARAK = 3;     // Toleransi error jarak maju-mundur (cm)

  // Gain untuk menjaga jarak & keseimbangan saat geser
  const float KP_DIST = 1.5;            // Gain untuk koreksi jarak (y)
  const float KP_BAL = 0.8;             // Gain untuk koreksi keseimbangan (z)
  const int MAX_CORRECTION_SPEED = 40;  // Batas kecepatan koreksi

  // --- State 1: Maju awal ---
  if (robotState == 1) {
    x = 0;
    y = 150;
    z = 0;
    if (pos_y >= 650) {
      robotState = 2;
      Serial.println("State 1 Selesai. Mulai berputar.");
    }

    // --- State 2: Berputar ke heading target ---
  } else if (robotState == 2) {
    const float TARGET_HEADING = -88;
    const float KP = 0.7;       // Gain
    const int TOLERANSI = 1;    // Range heading
    const int MAX_SPEED = 150;  // Z max speed

    float error = TARGET_HEADING - heading;

    if (abs(error) <= TOLERANSI) {
      x = 0;
      y = 0;
      z = 0;
      Odometry1 = 0;
      Odometry2 = 0;
      pos_y = 0;  // reset pos_y agar maju di state 3 presisi
      pos_x = 0;
      robotState = 3;

      Serial.print("Target heading tercapai di: ");
      Serial.println(heading);
    } else {
      z = KP * error;
      z = constrain(z, -MAX_SPEED, MAX_SPEED);

      // min speed supaya robot tetap bergerak
      if (abs(z) < 90) {
        if (z > 0) z = 80;
        if (z < 0) z = -80;
      }

      x = 0;
      y = 0;
      // // z sudah dihitung di atas

      Serial.print("Menuju target... Heading: ");
      Serial.print(heading);
      Serial.print(" | Error: ");
      Serial.print(error);
      Serial.print(" | Kecepatan Putar (z): ");
      Serial.println(z);
    }
  } else if (robotState == 3) {
    x = 0;
    y = 100;
    z = 0;
    // Cek jika SALAH SATU sensor sudah <= target jarak
    if (readLidar[0] <= TARGET_JARAK || readLidar[1] <= TARGET_JARAK) {
      y = 0;  // Stop maju
      robotState = 4;
      Serial.println("Objek terdeteksi. Masuk mode penyeimbang (State 4).");
    }

    // --- State 4: Menyeimbangkan posisi (tegak lurus halangan) ---
  } else if (robotState == 4) {
    x = 0;
    y = 0;
    int diff = readLidar[0] - readLidar[1];  // Selisih kiri - kanan

    if (abs(diff) <= TOLERANSI_SEIMBANG) {
      robotState = 5;  // Sudah seimbang, mulai geser
      pos_x = 0;       // Reset pos_x untuk state selanjutnya
      z = 0;
      Serial.println("Seimbang! Mulai geser kanan (State 5).");
    } else {
      // Kontrol proporsional untuk berputar
      z = KP_BAL * diff;
      z = constrain(z, -60, 60);  
    }

    // --- State 5 & 6: Geser Kanan/Kiri dengan Penjaga Jarak & Keseimbangan ---
  } else if (robotState == 5 || robotState == 6) {
    // 1. Tentukan Arah Gerak Utama (X)
    if (robotState == 5) {  // Geser Kanan
      x = 100;
      if (pos_x >= 200) {
        robotState = 6;
        Serial.println("Ujung kanan tercapai. Balik ke kiri (State 6).");
      }
    } else {  // Geser Kiri (State 6)
      x = -100;
      if (pos_x <= 0) {
        robotState = 5;
        Serial.println("Ujung kiri tercapai. Balik ke kanan (State 5).");
      }
    }

    // 2. Hitung Koreksi Jarak (Y) dan Keseimbangan (Z)
    int avgDist = (readLidar[0] + readLidar[1]) / 2;
    int distError = avgDist - TARGET_JARAK;
    int balError = readLidar[0] - readLidar[1];

    // Koreksi maju/mundur agar tetap di TARGET_JARAK
    if (abs(distError) > TOLERANSI_JARAK) {
      y = KP_DIST * distError;
    } else {
      y = 0;
    }

    // Koreksi putaran agar tetap tegak lurus
    if (abs(balError) > TOLERANSI_SEIMBANG) {
      z = KP_BAL * balError;
    } else {
      z = 0;
    }

    // Batasi kecepatan koreksi agar tidak mengganggu gerak utama X
    y = constrain(y, -MAX_CORRECTION_SPEED, MAX_CORRECTION_SPEED);
    z = constrain(z, -MAX_CORRECTION_SPEED, MAX_CORRECTION_SPEED);
  }
}

// ======================= LIDAR ========================== //
void Lidar_Read() {
  readLidar[0] = lidar[0].read();
  readLidar[1] = lidar[1].read();
}

void Debug_Lidar() {
  Lidar_Read();

  Serial.print("Lidar 1 : ");
  Serial.print(readLidar[0]);
  Serial.print("  Lidar 2 :");
  Serial.print(readLidar[1]);
  Serial.print("  compass :");
  Serial.println(heading);
}