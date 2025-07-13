void setup() {
  initial_setup();
}

void loop() {
  // =========== Gerakan XYZ ============
  float x = 0;
  float y = 0;
  float z = 1;
  holonomic(x, y, -z);
  Serial.print(encoderMotor1);
  Serial.print("  ");
  Serial.print(encoderMotor2);
  Serial.print("  ");
  Serial.print(encoderMotor3);
  Serial.print("  ");
  Serial.println(encoderMotor4);
}