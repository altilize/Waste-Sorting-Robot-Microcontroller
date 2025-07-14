void setup() {
  initial_setup();
  
}

void loop() {
  // =========== Gerakan XYZ ============
  float x = 0;
  float y = 2;
  float z = 0;
  holonomic(x, y, -z);
  // Serial.print(Odometry1);
  // Serial.print("  ");
  // Serial.print(Odometry2);
  // Serial.print("  ");
  // Serial.print(Odometry3);
  // Serial.print("  ");
  // Serial.println("");
}