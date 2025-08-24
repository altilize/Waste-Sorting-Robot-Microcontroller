// ---------- Process and List Command ------ //
typedef struct {
  const char *cmd;
  void (*handler)(char *args);
} Command;

Command commands[] = {
  { "A", handle_arm },
  { "AH", handle_home_arm },
  { "M", handle_movement },
  { "S", handle_suction },
  { "R", handle_lidar },
  { "RAW_LF", handle_raw_lf },
  { "DIG_LF", handle_dig_lf },
  { "?", handle_query },
  {"GO", handle_compass},
};

void process_command(char *command) {
  char *cmd = strtok(command, " ");
  char *args = strtok(NULL, "");

  for (size_t i = 0; i < sizeof(commands) / sizeof(commands[0]); i++) {
    if (strcmp(cmd, commands[i].cmd) == 0) {
      commands[i].handler(args);
      return;
    }
  }
  Serial.println("Unknown command. Please try again.");
}

void read_command() {
  char command[100];
  if (Serial.available()) {
    int len = Serial.readBytesUntil('\n', command, sizeof(command) - 1);
    command[len] = '\0';       // Null-terminate the string
    process_command(command);  // Handle other commands
  }
}

// ---------- Command A - Arm [pos] ------------ //
void handle_arm(char *args) {
  static int ARM_TARGET_POS = 0;

  if (sscanf(args, "%d", &ARM_TARGET_POS) == 1) {
    arm_target_position = ARM_TARGET_POS;

    Serial.print("Arm target position set to: ");
    Serial.println(arm_target_position);
  } else {
    Serial.println("Invalid command format. Use: A (position)");
  }
}

// ---------- Command AH - Arm Home ------------ /
void handle_home_arm(char *args) {
  if (arm_homed) return;

  unsigned long start_time = millis();
  const unsigned long timeout = 5000;

  while (digitalRead(ARM_LIMIT) != LOW) {
    analogWrite(ARM_FORWARD_PIN, 80);
    analogWrite(ARM_BACKWARD_PIN, 0);

    if (millis() - start_time > timeout) {
      analogWrite(ARM_FORWARD_PIN, 0);
      analogWrite(ARM_BACKWARD_PIN, 0);
      return;
    }
  }

  analogWrite(ARM_FORWARD_PIN, 0);
  analogWrite(ARM_BACKWARD_PIN, 0);
  encoderarm_count = 0;
  arm_homed = true;
}

// ---------- Command M [x] [y] [z] - Holonomic x,y and z ------------ /
void handle_movement(char *args) {
  int parsedVx, parsedVy, parsedVw;

  if (sscanf(args, "%d %d %d", &parsedVx, &parsedVy, &parsedVw) == 3) {
    x = parsedVy;
    y = parsedVx;
    z = parsedVw;

    // Debugging: Print the parsed values
    Serial.print("Parsed velocities - vx: ");
    Serial.print(x);
    Serial.print(", vy: ");
    Serial.print(y);
    Serial.print(", vw: ");
    Serial.println(z);

    holonomic(x, y, z);
  } else {
    Serial.println("Invalid command format. Use: M vx vy vw");
  }
}

// ------------- Command S - Toggle Suction ------------ //
void handle_suction(char *args) {
  
  suction_state = !suction_state;

  if (suction_state) {
    digitalWrite(SUCTION_PIN, HIGH);
    Serial.println("Suction turned ON");
  } else {
    digitalWrite(SUCTION_PIN, LOW);
    Serial.println("Suction turned OFF");
  }

}

// ----------- Command R - Debug Lidar ------------------ //
// must run Lidar_Read();
void handle_lidar(char *args) {
    Serial.print("L:");
    Serial.print(readLidar[0]);
    Serial.print(" R:");
    Serial.println(readLidar[1]);
}

// ------- Command RAW_LF - Debug LF (Analog Values) ----- //
// must run LF_Read();
void handle_raw_lf(char *args) {
    // Serial.print("Hor: ");
    // for (int i = 0; i < 8; i++) {
    //     Serial.print(raw_sensor_horizontal[i]);
    //     Serial.print(" ");
    // }
    // Serial.println();
    Serial.print("Ver: ");
    for (int i = 0; i < 6; i++) {
        Serial.print(LF_Vertikal[i]);
        Serial.print(" ");
    }
    Serial.println();
}

// ------- Command DIG_LF - Debug LF (Digital Values) ------- //
// must run LF_WeightedAverage();
void handle_dig_lf(char *args) {
    // Serial.println("Digital Line Follower Sensor Values:");
    // Serial.print("Hor: ");
    // for (int i = 0; i < 8; i++) {
    //     Serial.print(digital_sensor_hor[i]);
    //     Serial.print(" ");
    // }
    // Serial.println();
    Serial.print("Vert: ");
    for (int i = 0; i < 6; i++) {
        Serial.print(LF_Vertikal_Dig[i]);
        Serial.print(" ");
    }
    Serial.println();
}

// -------- Command ? - Debug Everything (not really) -------- //
void handle_query(char *args) { 
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

// ------- Command GO - Run Home to Conveyor + print value initial compass ---- //
void handle_compass(char *args) {
  robotState = 1;
  Serial.println(heading);
}

