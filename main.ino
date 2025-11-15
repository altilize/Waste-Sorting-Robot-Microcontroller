void setup() {
  Serial.begin(115200);
  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
  // analogWriteFrequency(20000);  // biar motor ga bunyi
  initial_setup();
}

void loop() {
  calculate_position();  // menghitung pos_x dan pos_y
  read_compass();
  Lidar_Read();
  read_command();


  if (digitalRead(BUTTON) == HIGH) {
    // reset Motor
    x = 0;
    y = 0;
    z = 0;
    SpeedA = 0, SpeedB = 0, SpeedC = 0, SpeedD = 0;
    PID_A = 0, PID_B = 0, PID_C = 0, PID_D = 0;
    // reset Odometry
    Odometry1 = 0;
    Odometry2 = 0;
    Odometry3 = 0;
    pos_x = 0.0;
    pos_y = 0.0;
    digitalWrite(ENABLE_MOTOR_PIN, LOW);
  } else {
    if (robotState != 0) HomeToConveyor();
    holonomic(x, y, -z);
  }
  
  // Debug_LF_Vertikal();
  // Serial.println(RE);
  // Debug_Lidar();
  // Serial.println(heading);
  

}





















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