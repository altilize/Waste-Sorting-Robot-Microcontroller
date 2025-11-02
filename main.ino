void setup() {
  Serial.begin(115200);
  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
  // analogWriteFrequency(20000);  // biar motor ga bunyi
  initial_setup();
}

void loop() {
  // ---- Input -----
  // calculate_position();  // menghitung pos_x dan pos_y
  // read_compass();
  // // Lidar_Read();
  // Debug_Lidar();
  // Serial.print("LS 1 : ");
  // Serial.print(digitalRead(LimitSwitch_1));
  // Serial.print("  Odo 2 : ");
  // Serial.print(digitalRead(LimitSwitch_2));
  // Serial.print("  Odo 3 : ");
  // Serial.print(digitalRead(BUTTON));
  // Serial.println();
  // digitalWrite(ENABLE_MOTOR_PIN, HIGH);


  Serial.print(encoder1RPM);
  Serial.print(" ");
  Serial.print(encoder2RPM);
  Serial.print(" ");
  Serial.print(encoder3RPM);
  Serial.print(" ");
  Serial.println(encoder4RPM);
  // LF_Read();
  // Debug_LF_Vertikal();


  read_command();  // baca command
  holonomic(x, y, -z);

  // holonomic(x, y, -z);
  // Serial.println(heading);
}






















// -----------------------
// 2. Tes Motor A MAJU (Forward) dengan kecepatan 50
// analogWrite(AmotorL_PIN, 50);    // Beri PWM 50 ke pin Kiri
// digitalWrite(AmotorR_PIN, LOW);  // Set pin Kanan ke LOW
// analogWrite(BmotorL_PIN, 50);    // Beri PWM 50 ke pin Kiri
// digitalWrite(BmotorR_PIN, LOW);  // Set pin Kanan ke LOW
// analogWrite(CmotorL_PIN, 50);    // Beri PWM 50 ke pin Kiri
// digitalWrite(CmotorR_PIN, LOW);  // Set pin Kanan ke LOW
// analogWrite(DmotorL_PIN, 50);    // Beri PWM 50 ke pin Kiri
// digitalWrite(DmotorR_PIN, LOW);  // Set pin Kanan ke LOW
// // -------------- nyoba lucu lucuan aj ---------- //
// if (robotState == 1) {

//   HomeToConveyor();

//   if (pos_y >= 200) {
//     robotState = 2;
//     Serial.println("Posisi y=500 tercapai. Mulai berputar.");
//   }
// } else if (robotState == 2) {
//   const float TARGET_HEADING = -88;
//   const float KP = 0.7;               // Gain
//   const int TOLERANSI = 1;            // Range heading
//   const int MAX_SPEED = 90;           // Z max speed

//   float error = TARGET_HEADING - heading;

//   if (abs(error) <= TOLERANSI) {
//       x = 0;
//       y = 0;
//       z = 0;
//       robotState = 3;
//       Serial.print("Target heading tercapai di: ");
//       Serial.println(heading);
//   } else {
//       z = KP * error;
//       z = constrain(z, -MAX_SPEED, MAX_SPEED);

//       // min speed supaya robot tetap bergerak
//       if (abs(z) < 20) {
//         if (z > 0) z = 20;
//         if (z < 0) z = -20;
//       }

//       x = 0;
//       y = 0;
//       // // z sudah dihitung di atas

//       Serial.print("Menuju target... Heading: ");
//       Serial.print(heading);
//       Serial.print(" | Error: ");
//       Serial.print(error);
//       Serial.print(" | Kecepatan Putar (z): ");
//       Serial.println(z);
//   }  digitalWrite(ENABLE_MOTOR_PIN, HIGH);
// } else if (robotState == 3) {
//   // STATE 2: SELESAI
//   x = 0;
//   y = 0;
//   z = 0;
// }
// z = 0;
// x = 0;
// y = 0;