void setup() {
  Serial.begin(115200);
  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
  analogWriteFrequency(20000);  // biar motor ga bunyi
  initial_setup();
}

void loop() {
  // =========== Gerakan XYZ ============
  float x = 0;
  float y = -2;
  float z = 0;
 // holonomic(x, y, -z);
 Debug_Lidar();
}