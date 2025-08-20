void handle_query() {       // command == '?'
  Serial.print(Odometry1);  // Rotary 1-3
  Serial.print(" ");
  Serial.print(Odometry2);
  Serial.print(" ");
  Serial.print(Odometry3);
  Serial.print("  ");
  Serial.print(encoder1RPM);  // Encoder Motor 1-4
  Serial.print(" ");
  Serial.print(encoder2RPM);
  Serial.print(" ");
  Serial.print(encoder3RPM);
  Serial.print(" ");
  Serial.print(encoder4RPM);
  Serial.print("  ");
  Serial.print(pos_x);
  Serial.print(" ");
  Serial.print(pos_y);
  Serial.println(" ");  // nanti kalo dah ada kompas tambahin thetanya juga
}

void handle_movement() {
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
}
void read_command() {
  if (Serial.available() > 0) {
    command = Serial.readStringUntil('\n');
    command.trim();

    // ------------ Command Handle ----------------
    if (command == "?") {
      handle_query();
    } else if (command.startsWith("M") || command.startsWith("m")) {
      handle_movement();

    } else if (command == "START") {
      START = !START;
    } else if (command == "LF") {
      isLF_ON = !isLF_ON;
      Serial.print("Line Follower Status : ");
      Serial.println(isLF_ON);
    } else if (command == "RES") {
      ser_RESET = true;
      delay(10);
      ser_RESET = false;
      Serial.print("RESET");
    }
  }
}