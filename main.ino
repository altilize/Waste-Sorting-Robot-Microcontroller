void setup() {
  Serial.begin(115200);
  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
  // analogWriteFrequency(20000);  // biar motor ga bunyi
  initial_setup();
}

void loop() {
  // =========== Gerakan XYZ ============
  float x = 0;
  float y = 2;
  float z = 0;
  holonomic(x, y, -z);
  //  Debug_Lidar();
  // Serial.print(encoderMotor1);
  // Serial.print(" ");
  // Serial.print(encoderMotor2);
  // Serial.print(" ");
  // Serial.print(encoderMotor3);
  // Serial.print(" ");
  // Serial.print(encoderMotor4);
  // Serial.println(" ");

  // 1 (z = 135) (-z = 135)
  // 2 (z = 138) (-z = 269)
  // 3 (z = 143)  (-z = 267)
  // 4 (z = 132) (-z = 134)

  Serial.print(rpmA);
  Serial.print(" ");
  Serial.print(rpmB);
  Serial.print(" ");
  Serial.print(rpmC);
  Serial.print(" ");
  Serial.print(rpmD);
  Serial.println();
}