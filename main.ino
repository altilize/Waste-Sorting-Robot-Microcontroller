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




  // =============== BNN (Bagian Ngedebug - Ngedebug) ====================== 
  //  Debug_Lidar();
  // Serial.print(encoderMotor1);
  // Serial.print(" ");
  // Serial.print(encoderMotor2);
  // Serial.print(" ");
  // Serial.print(encoderMotor3);
  // Serial.print(" ");
  // Serial.print(encoderMotor4);
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