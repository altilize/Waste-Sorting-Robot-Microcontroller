void read_command() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    // Hapus spasi di awal dan akhir String
    command.trim();

    if (command == "INFO") {
      // Kirim nilai pos_x dan pos_y ke Serial Monitor
      Serial.print("X : ");
      Serial.print(pos_x);
      Serial.print("Y : ");
      Serial.print(pos_y);
      Serial.print("  M1 : ");
      Serial.print(encoderMotor1);
      Serial.print(" M2 : ");
      Serial.print(encoderMotor2);
      Serial.print(" M3 : ");
      Serial.print(encoderMotor3);
      Serial.print(" M4 : ");
      Serial.println(encoderMotor4);
    } else if (command.startsWith("M")) {

      int firstSpace = command.indexOf(' ');

      String params = command.substring(firstSpace + 1);

      int secondSpace = params.indexOf(' ');
      String vx_str = params.substring(0, secondSpace);

      params = params.substring(secondSpace + 1);


      int thirdSpace = params.indexOf(' ');
      String vy_str = params.substring(0, thirdSpace);

      String vz_str = params.substring(thirdSpace + 1);

      x = vx_str.toFloat();
      y = vy_str.toFloat();
      z = vz_str.toFloat();

    } else if (command == "START") {
      START = !START;
    }
  }
}