// ======================== LINE FOLLOWER ======================
void LF_Read() {
  MUX_Select(0);
  delayMicroseconds(20);
  LF_Vertikal[0] = analogRead(MUX_INPUT);

  MUX_Select(1);
  delayMicroseconds(20);
  LF_Vertikal[1] = analogRead(MUX_INPUT);

  MUX_Select(2);
  delayMicroseconds(20);
  LF_Vertikal[2] = analogRead(MUX_INPUT);

  MUX_Select(3);
  delayMicroseconds(20);
  LF_Vertikal[3] = analogRead(MUX_INPUT);

  MUX_Select(4);
  delayMicroseconds(20);
  LF_Vertikal[4] = analogRead(MUX_INPUT);

  MUX_Select(5);
  delayMicroseconds(20);
  LF_Vertikal[5] = analogRead(MUX_INPUT);

  MUX_Select(6);
  delayMicroseconds(20);
  LF_Vertikal[6] = analogRead(MUX_INPUT);

  MUX_Select(7);
  delayMicroseconds(20);
  LF_Vertikal[7] = analogRead(MUX_INPUT);
}

float LF_WeightedAverage() {
  float TotalWeight = 0;
  int TotalActiveSensor = 0;
  LF_Read();

  if (LF_Vertikal[0] >= 650) {
    TotalWeight += weights[0];
    TotalActiveSensor++;
  }

  if (LF_Vertikal[1] >= 650) {
    TotalWeight += weights[1];
    TotalActiveSensor++;
  }

  if (LF_Vertikal[2] >= 650) {
    TotalWeight += weights[2];
    TotalActiveSensor++;
  }

  if (LF_Vertikal[3] >= 650) {
    TotalWeight += weights[3];
    TotalActiveSensor++;
  }
  //  -----------------------------
  if (LF_Vertikal[4] >= 650) {
    TotalWeight += weights[4];
    TotalActiveSensor++;
  }
  if (LF_Vertikal[5] >= 650) {
    TotalWeight += weights[5];
    TotalActiveSensor++;
  }
  if (LF_Vertikal[6] >= 650) {
    TotalWeight += weights[6];
    TotalActiveSensor++;
  }
  // ------------------------------

  // Jika tidak ada garis
  if (TotalActiveSensor == 0) {
    return 0;
  }

  return TotalWeight / TotalActiveSensor;
}


//  ============== Debugging Line Follower ================ //
void Debug_LF_Vertikal() {
  float WA = LF_WeightedAverage();

  Serial.print(LF_Vertikal[0]);
  Serial.print(" ");
  Serial.print(LF_Vertikal[1]);
  Serial.print(" ");
  Serial.print(LF_Vertikal[2]);
  Serial.print(" ");
  Serial.print(LF_Vertikal[3]);
  Serial.print(" ");
  Serial.print(LF_Vertikal[4]);
  Serial.print(" ");
  Serial.print(LF_Vertikal[5]);
  Serial.print(" ");
  Serial.print(LF_Vertikal[6]);
  Serial.print(" ");
  Serial.print(LF_Vertikal[7]);
  Serial.print("  | Output : ");
  Serial.println(WA);
}

void HomeToConveyor() {
  if (pos_y >= 0 && pos_y <= 500) {
    x = 0;
    y = 100;
    z = 0;
  } else {
    x = 0, y = 0, z = 0;
  }
}


// ======================= LIDAR ========================== //
void Lidar_Read() {
  readLidar[0] = lidar[0].read();
  readLidar[1] = lidar[1].read();
}

void Debug_Lidar() {
  Lidar_Read();

  Serial.print(readLidar[0]);
  Serial.print(" ");
  Serial.print(readLidar[1]);

  Serial.println(" ");
}