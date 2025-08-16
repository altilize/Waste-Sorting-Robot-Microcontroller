void setup() {
  Serial.begin(115200);
  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
  analogWriteFrequency(20000);  // biar motor ga bunyi
  initial_setup();
}

void loop() {
  calculate_position();  // menghitung pos_x dan pos_y
  read_command();        //baca command
  if (START) {
    holonomic(x, y, -z);
  } else {
    digitalWrite(Enable, LOW);
    SpeedA = 0, SpeedB = 0, SpeedC = 0, SpeedD = 0;
    PID_A = 0, PID_B = 0, PID_C = 0, PID_D = 0;
  }



  // =============== BNN (Bagian Ngedebug - Ngedebug) ======================
  //  Debug_Lidar();
  // Serial.print(Odometry1);
  // Serial.print(" ");
  // Serial.print(Odometry2);
  // Serial.print(" ");
  // Serial.print(Odometry3);
  // Serial.println(" ");

  Serial.print(rpmA);
  Serial.print(" ");
  Serial.print(rpmB);
  Serial.print(" ");
  Serial.print(rpmC);
  Serial.print(" ");
  Serial.print(rpmD);
  Serial.println();
}