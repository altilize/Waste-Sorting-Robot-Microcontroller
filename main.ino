void setup() {
  Serial.begin(115200);
  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
  analogWriteFrequency(20000);  // biar motor ga bunyi
  initial_setup();
}

void loop() {
  // ---- Input -----
  calculate_position();  // menghitung pos_x dan pos_y
  read_compass();
  Lidar_Read();
  read_command();  // baca command
  // ---- Output -----
  // controlArm();  // handle_arm - A [pos]
  // handle_home_arm();  // nanti di revisi - AH
  // handle_movement();  // nanti di cek lagi


// -------------- nyoba lucu lucuan aj ---------- //
  if (robotState == 1) {

    HomeToConveyor();

    if (pos_y >= 500) {
      robotState = 2;
      Serial.println("Posisi y=500 tercapai. Mulai berputar.");
    }
  } else if (robotState == 2) {
    // STATE 1: BERPUTAR
    if (heading > -70) {  // nanti di tambahin proteksi misal lebih dari -75 nanti z = 90 gitu lah xixixi
      x = 0;
      y = 0;
      z = -90;  
    } else {

      x = 0;
      y = 0;
      z = 0;
      robotState = 3;  // Ganti state menjadi "Selesai"
      Serial.println("Target heading -90 tercapai. Berhenti.");
    }
  } else if (robotState == 3) {
    // STATE 2: SELESAI
    x = 0;
    y = 0;
    z = 0;
  }

  holonomic(x, y, -z);
}