/*
  Authored by
  Moses Jaguar
  622023006

  -------------------------
  |   Setup Tools STM32   | 
  -------------------------
  Board Name : STM32F407Vetx
  Upload Method : SWD
  USB Support : CDC Generic Serial Supersede U(S)ART
  U(S)ART Support : Enabled Generic Serial

  See How to setup STM32DUINO on Ubuntu : 
  https://github.com/altilize/STM32Duino-on-Ubuntu

  ------------------------
    Kalau port ga kebaca |
  -----------------------
  Colokin port
  ls -l /dev/ttyACM*
  sudo chmod 666 /dev/ttyACM0 (tergantung nama port)
  sudo usermod -a -G dialout $USER

  newgrp dialout (opsional)
*/

// ---------- COBAAKK -----
bool START = false;
bool ser_RESET = false;
bool kntl = false;
int x = 0, y = 0, z = 0;

// ============= Serial ================= //
String command;

// ============ Motor ==================== //
#define AmotorL_PIN PA0
#define AmotorR_PIN PA2
#define BmotorR_PIN PA3
#define BmotorL_PIN PA1
#define CmotorL_PIN PA6  // PA6
#define CmotorR_PIN PB0  // PB0
#define DmotorL_PIN PA7  // PB1
#define DmotorR_PIN PB1  // PA7
#define ENABLE_MOTOR_PIN PB2

// #define EMERGENCY PC12
// ============== Encoder Motor ==============
#define AmotorENC_PIN PA5  // PD12 ubah ke PA5
#define BmotorENC_PIN PC6
#define DmotorENC_PIN PC4  // PD13 ubah ke PC4
#define CmotorENC_PIN PC7
volatile int encoderMotor1 = 0, encoderMotor2 = 0, encoderMotor3 = 0, encoderMotor4 = 0;
volatile int encoder1RPM = 0, encoder2RPM = 0, encoder3RPM = 0, encoder4RPM = 0;

// ----------- PID ----------
float setpoint1 = 0, setpoint2 = 0, setpoint3 = 0, setpoint4 = 0;
int rpmA = 0, rpmB = 0, rpmC = 0, rpmD = 0;
bool flagPID1 = false, flagPID2 = false, flagPID3 = false, flagPID4 = false;
float PID_A = 0, PID_B = 0, PID_C = 0, PID_D = 0;
float SpeedA = 0, SpeedB = 0, SpeedC = 0, SpeedD = 0;

// ============ Odometry ===================== //
#define ODOMETRY_PIN_1B PB10
#define ODOMETRY_PIN_1A PB12
#define ODOMETRY_PIN_2A PB11
#define ODOMETRY_PIN_2B PB13
#define ODOMETRY_PIN_3B PB15
#define ODOMETRY_PIN_3A PD9
volatile int Odometry1 = 0, Odometry2 = 0, Odometry3 = 0;
int pos_x, pos_y;  // BELUM

// ============= Arm ========================= //
#define ARM_FORWARD_PIN PA10
#define ARM_BACKWARD_PIN PE14
#define ARM_ENCODER_A_PIN PC8
#define ARM_ENCODER_B_PIN PA8
#define ARM_LIMIT PE3
int arm_target_position = 0;
bool arm_homed = false;
volatile long encoderarm_count = 0;

// ============= Suction ===================== //
#define SUCTION_PIN PD2
bool suction_state = false;

// ============ Multiplexer ================== //
const int MUX_Selektor_0 = PC0;
const int MUX_Selektor_1 = PC1;
const int MUX_Selektor_2 = PC2;
const int MUX_Selektor_3 = PC3;
const int MUX_INPUT = PA4;

// ------- Fungsi untuk membaca Mux (0-15) --------
void MUX_Select(int port) {
  if (port < 0 || port > 15) return;

  digitalWrite(MUX_Selektor_0, port & 1);
  digitalWrite(MUX_Selektor_1, (port >> 1) & 1);
  digitalWrite(MUX_Selektor_2, (port >> 2) & 1);
  digitalWrite(MUX_Selektor_3, (port >> 3) & 1);
}
// ------------- Line Follower ---------------------
volatile int LF_Vertikal[16];
volatile int LF_Vertikal_Dig[16];
float weights[] = { 30, 20, 10, 0, -10, -20, -30 };
bool isLF_ON = true;


// ============== Variabel Lidar =================== //
#include <VL53L1X.h>

const uint8_t xshutPins[2] = { PB5, PB3 };
VL53L1X lidar[2];
double readLidar[2];

// =============== Variabel Compass BNO055 ========== //
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
float heading;
int robotState = 0;
float headingOffset = 0.0;


// ==================== Limit Switch =======================
const int LimitSwitch_1 = PE3;
const int LimitSwitch_2 = PE5;
const int LimitSwitch_3 = PE1;
const int BUTTON = PE6;

// ============= Interupt Encoder Motor =====================
void ISR_encoder1() {
  encoderMotor1++;
}
void ISR_encoder2() {
  encoderMotor2++;
}
void ISR_encoder3() {
  encoderMotor3++;
}
void ISR_encoder4() {
  encoderMotor4++;
}

// ============== Interrupt Encoder Arm ======================
void ISR_encoderARM() {
  if (digitalRead(ARM_ENCODER_A_PIN) == digitalRead(ARM_ENCODER_B_PIN)) {
    encoderarm_count--;
  } else {
    encoderarm_count++;
  }
}

// ============== Interrupt Rotary Encoder ==================
void encA() {
  (digitalRead(ODOMETRY_PIN_1A) == digitalRead(ODOMETRY_PIN_1B)) ? Odometry1-- : Odometry1++;
}
void encB() {
  (digitalRead(ODOMETRY_PIN_2A) == digitalRead(ODOMETRY_PIN_2B)) ? Odometry2++ : Odometry2--;
}
void encC() {
  (digitalRead(ODOMETRY_PIN_3A) == digitalRead(ODOMETRY_PIN_3B)) ? Odometry3++ : Odometry3--;
}


// ============= Interrupt Timer  ============================
void updateoverflow() {
  encoder1RPM = encoderMotor1;
  encoder2RPM = encoderMotor2;
  encoder3RPM = encoderMotor3;
  encoder4RPM = encoderMotor4;

  encoderMotor1 = 0;
  encoderMotor2 = 0;
  encoderMotor3 = 0;
  encoderMotor4 = 0;

  rpmA = (encoder1RPM * 60 * 10) / 134;  // Encoder * 60 detik/ 0.1 detik (interval 100ms) * (ppr * rasio motor)
  rpmB = (encoder2RPM * 60 * 10) / 134;
  rpmC = (encoder3RPM * 60 * 10) / 134;
  rpmD = (encoder4RPM * 60 * 10) / 134;

  pid();
  // pid(); kalau mau PIDnya setiap 10 Hz, harus di bagi interval di function PIDnya

  // Serial.print(rpmA);
  // Serial.print("|");
  // Serial.print(rpmB);
  // Serial.print("|");
  // Serial.print(rpmC);
  // Serial.print("|");
  // Serial.print(rpmD);
  // Serial.println();
}
// ======================== LINE FOLLOWER ======================
void LF_Read() {
  MUX_Select(0);
  delayMicroseconds(100);
  LF_Vertikal[0] = analogRead(MUX_INPUT);

  MUX_Select(1);
  delayMicroseconds(100);
  LF_Vertikal[1] = analogRead(MUX_INPUT);

  MUX_Select(2);
  delayMicroseconds(100);
  LF_Vertikal[2] = analogRead(MUX_INPUT);

  MUX_Select(3);
  delayMicroseconds(100);
  LF_Vertikal[3] = analogRead(MUX_INPUT);

  MUX_Select(4);
  delayMicroseconds(100);
  LF_Vertikal[4] = analogRead(MUX_INPUT);

  MUX_Select(5);
  delayMicroseconds(100);
  LF_Vertikal[5] = analogRead(MUX_INPUT);

  MUX_Select(6);
  delayMicroseconds(100);
  LF_Vertikal[6] = analogRead(MUX_INPUT);

  MUX_Select(7);
  delayMicroseconds(100);
  LF_Vertikal[7] = analogRead(MUX_INPUT);

  MUX_Select(8);
  delayMicroseconds(100);
  LF_Vertikal[8] = analogRead(MUX_INPUT);

  MUX_Select(9);
  delayMicroseconds(100);
  LF_Vertikal[9] = analogRead(MUX_INPUT);

  MUX_Select(10);
  delayMicroseconds(100);
  LF_Vertikal[10] = analogRead(MUX_INPUT);

  MUX_Select(11);
  delayMicroseconds(100);
  LF_Vertikal[11] = analogRead(MUX_INPUT);

  MUX_Select(12);
  delayMicroseconds(100);
  LF_Vertikal[12] = analogRead(MUX_INPUT);

  MUX_Select(13);
  delayMicroseconds(100);
  LF_Vertikal[13] = analogRead(MUX_INPUT);

  MUX_Select(14);
  delayMicroseconds(100);
  LF_Vertikal[14] = analogRead(MUX_INPUT);
}

float LF_WeightedAverage() {
  float TotalWeight = 0;
  int TotalActiveSensor = 0;
  LF_Read();
  // reset values
  for (int i = 0; i <= 6; i++) {
    LF_Vertikal_Dig[i] = 0;
  }

  if (LF_Vertikal[0] >= 650) {
    TotalWeight += weights[0];
    TotalActiveSensor++;
    LF_Vertikal_Dig[0] = 1;
  }

  if (LF_Vertikal[1] >= 650) {
    TotalWeight += weights[1];
    TotalActiveSensor++;
    LF_Vertikal_Dig[1] = 1;
  }

  if (LF_Vertikal[2] >= 650) {
    TotalWeight += weights[2];
    TotalActiveSensor++;
    LF_Vertikal_Dig[2] = 1;
  }

  if (LF_Vertikal[3] >= 650) {
    TotalWeight += weights[3];
    TotalActiveSensor++;
    LF_Vertikal_Dig[3] = 1;
  }
  if (LF_Vertikal[4] >= 650) {
    TotalWeight += weights[4];
    TotalActiveSensor++;
    LF_Vertikal_Dig[4] = 1;
  }
  if (LF_Vertikal[5] >= 650) {
    TotalWeight += weights[5];
    TotalActiveSensor++;
    LF_Vertikal_Dig[5] = 1;
  }
  if (LF_Vertikal[6] >= 650) {
    TotalWeight += weights[6];
    TotalActiveSensor++;
    LF_Vertikal_Dig[6] = 1;
  }

  // Jika tidak ada garis
  if (TotalActiveSensor == 0) {
    return 0;
  }

  return TotalWeight / TotalActiveSensor;
}


//  ============== Debugging Line Follower ================ //
void Debug_LF_Vertikal() {
  float WA = LF_WeightedAverage();

  for (int i = 0; i < 15; i++) {
    if (LF_Vertikal[i] >= 400) {
      LF_Vertikal_Dig[i] = 1;
    } else {
      LF_Vertikal_Dig[i] = 0;
    }
    Serial.print(LF_Vertikal_Dig[i]);
    Serial.print("  ");
  }
  Serial.println();
  // Serial.print(LF_Vertikal[0]);
  // Serial.print(" ");
  // Serial.print(LF_Vertikal[1]);
  // Serial.print(" ");
  // Serial.print(LF_Vertikal[2]);
  // Serial.print(" ");
  // Serial.print(LF_Vertikal[3]);
  // Serial.print(" ");
  // Serial.print(LF_Vertikal[4]);
  // Serial.print(" ");
  // Serial.print(LF_Vertikal[5]);
  // Serial.print(" ");
  // Serial.print(LF_Vertikal[6]);
  // Serial.print(" ");
  // Serial.print(LF_Vertikal[7]);
  // Serial.print(" ");
  // Serial.print(LF_Vertikal[8]);
  // Serial.print(" ");
  // Serial.print(LF_Vertikal[9]);
  // Serial.print(" ");
  // Serial.print(LF_Vertikal[10]);
  // Serial.print(" ");
  // Serial.print(LF_Vertikal[11]);
  // Serial.print(" ");
  // Serial.print(LF_Vertikal[12]);
  // Serial.print(" ");
  // Serial.print(LF_Vertikal[13]);
  // Serial.print(" ");
  // Serial.print(LF_Vertikal[14]);


  // Serial.print("  | Output : ");
  // Serial.println(WA);
}

// ---------------------- Utama Algoritma ------------------ //
void HomeToConveyor() {
  // --- Konstanta untuk Kontrol Lidar ---
  const int TARGET_JARAK = 140;      // Target jarak ke halangan (cm)
  const int TOLERANSI_SEIMBANG = 3;  // Toleransi beda jarak kiri-kanan (cm)
  const int TOLERANSI_JARAK = 3;     // Toleransi error jarak maju-mundur (cm)

  // Gain untuk menjaga jarak & keseimbangan saat geser
  const float KP_DIST = 1.5;            // Gain untuk koreksi jarak (y)
  const float KP_BAL = 0.8;             // Gain untuk koreksi keseimbangan (z)
  const int MAX_CORRECTION_SPEED = 40;  // Batas kecepatan koreksi
  // Serial.println(heading);

  // --- State 1: Maju awal ---
  if (robotState == 1) {
    x = 0;
    y = 150;
    z = 0;
    if (pos_y >= 650) {
      robotState = 2;
      // Serial.println("State 1 Selesai. Mulai berputar.");
    }

    // --- State 2: Berputar ke heading target ---
  } else if (robotState == 2) {
    const float TARGET_HEADING = -88;
    const float KP = 0.7;       // Gain
    const int TOLERANSI = 1;    // Range heading
    const int MAX_SPEED = 150;  // Z max speed

    float error = TARGET_HEADING - heading;

    if (abs(error) <= TOLERANSI) {
      x = 0;
      y = 0;
      z = 0;
      Odometry1 = 0;
      Odometry2 = 0;
      pos_y = 0;  // reset pos_y agar maju di state 3 presisi
      pos_x = 0;
      robotState = 3;

      // Serial.print("Target heading tercapai di: ");
      // Serial.println(heading);
    } else {
      z = KP * error;
      z = constrain(z, -MAX_SPEED, MAX_SPEED);

      // min speed supaya robot tetap bergerak
      if (abs(z) < 90) {
        if (z > 0) z = 80;
        if (z < 0) z = -80;
      }

      x = 0;
      y = 0;
      // // z sudah dihitung di atas

      // Serial.print("Menuju target... Heading: ");
      Serial.println(heading);
      // Serial.print(" | Error: ");
      // Serial.print(error);
      // Serial.print(" | Kecepatan Putar (z): ");
      // Serial.println(z);
    }
  } else if (robotState == 3) {
    x = 0;
    y = 100;
    z = 0;
    // Cek jika SALAH SATU sensor sudah <= target jarak
    if (readLidar[0] <= TARGET_JARAK || readLidar[1] <= TARGET_JARAK) {
      y = 0;  // Stop maju
      robotState = 4;
      // Serial.println("Objek terdeteksi. Masuk mode penyeimbang (State 4).");
    }

    // --- State 4: Menyeimbangkan posisi (tegak lurus halangan) ---
  } else if (robotState == 4) {
    x = 0;
    y = 0;
    int diff = readLidar[0] - readLidar[1];  // Selisih kiri - kanan

    if (abs(diff) <= TOLERANSI_SEIMBANG) {
      robotState = 5;  // Sudah seimbang, mulai geser
      pos_x = 0;       // Reset pos_x untuk state selanjutnya
      z = 0;
      // Serial.println("Seimbang! Mulai geser kanan (State 5).");
    } else {
      // Kontrol proporsional untuk berputar
      z = KP_BAL * diff;
      z = constrain(z, -60, 60);
    }

    // --- State 5 & 6: Geser Kanan/Kiri dengan Penjaga Jarak & Keseimbangan ---
  } else if (robotState == 5 || robotState == 6) {
    // 1. Tentukan Arah Gerak Utama (X)
    if (robotState == 5) {  // Geser Kanan
      x = 100;
      if (pos_x >= 200) {
        robotState = 6;
        // Serial.println("Ujung kanan tercapai. Balik ke kiri (State 6).");
      }
    } else {  // Geser Kiri (State 6)
      x = -100;
      if (pos_x <= 0) {
        robotState = 5;
        // Serial.println("Ujung kiri tercapai. Balik ke kanan (State 5).");
      }
    }

    // 2. Hitung Koreksi Jarak (Y) dan Keseimbangan (Z)
    int avgDist = (readLidar[0] + readLidar[1]) / 2;
    int distError = avgDist - TARGET_JARAK;
    int balError = readLidar[0] - readLidar[1];

    // Koreksi maju/mundur agar tetap di TARGET_JARAK
    if (abs(distError) > TOLERANSI_JARAK) {
      y = KP_DIST * distError;
    } else {
      y = 0;
    }

    // Koreksi putaran agar tetap tegak lurus
    if (abs(balError) > TOLERANSI_SEIMBANG) {
      z = KP_BAL * balError;
    } else {
      z = 0;
    }

    // Batasi kecepatan koreksi agar tidak mengganggu gerak utama X
    y = constrain(y, -40, 40);
    z = constrain(z, -40, 40);
  }
}

// ======================= LIDAR ========================== //
void Lidar_Read() {
  readLidar[0] = lidar[0].read();
  readLidar[1] = lidar[1].read();
}

void Debug_Lidar() {
  Lidar_Read();

  Serial.print("Lidar 1 : ");
  Serial.print(readLidar[0]);
  Serial.print("  Lidar 2 :");
  Serial.print(readLidar[1]);
  Serial.print("  compass :");
  Serial.println(heading);
}
// ======================= Holonomic Function ================ //
void holonomic(float vx, float vy, float vz) {



  float holonomic_speedA = (-0.35 * vx) + (0.35 * vy) + (0.25 * vz);
  float holonomic_speedB = (-0.35 * vx) + (-0.35 * vy) + (0.25 * vz);
  float holonomic_speedC = (0.35 * vx) + (-0.35 * vy) + (0.25 * vz);
  float holonomic_speedD = (0.35 * vx) + (0.35 * vy) + (0.25 * vz);

  setpoint1 = holonomic_speedA * 10;
  setpoint2 = holonomic_speedB * 10;
  setpoint3 = holonomic_speedC * 10;
  setpoint4 = holonomic_speedD * 10;

  // SpeedA = holonomic_speedA;
  // SpeedB = holonomic_speedB;
  // SpeedC = holonomic_speedC;
  // SpeedD = holonomic_speedD;


  flagPID1 = (setpoint1 < 0);
  flagPID2 = (setpoint2 < 0);
  flagPID3 = (setpoint3 < 0);
  flagPID4 = (setpoint4 < 0);

  setpoint1 = abs(setpoint1);
  setpoint2 = abs(setpoint2);
  setpoint3 = abs(setpoint3);
  setpoint4 = abs(setpoint4);

  // pid ada di updateoverflow(); 10hz
  // pid();

  motorauto();
}

void motorauto() {
  digitalWrite(ENABLE_MOTOR_PIN, HIGH);

  int pwmA = SpeedA;
  int pwmB = SpeedB;
  int pwmC = SpeedC;
  int pwmD = SpeedD;

  // -------- motor A ------------
  if (pwmA > 0) {
    analogWrite(AmotorL_PIN, pwmA);
    analogWrite(AmotorR_PIN, 0);
  } else {
    analogWrite(AmotorL_PIN, 0);
    analogWrite(AmotorR_PIN, abs(pwmA));
  }

  // -------- motor B ------------
  if (pwmB > 0) {
    analogWrite(BmotorL_PIN, pwmB);
    analogWrite(BmotorR_PIN, 0);
  } else {
    analogWrite(BmotorL_PIN, 0);
    analogWrite(BmotorR_PIN, abs(pwmB));
  }

  // -------- motor C ------------
  if (pwmC > 0) {
    analogWrite(CmotorL_PIN, pwmC);
    analogWrite(CmotorR_PIN, 0);
  } else {
    analogWrite(CmotorL_PIN, 0);
    analogWrite(CmotorR_PIN, abs(pwmC));
  }

  // -------- motor D ------------
  if (pwmD > 0) {
    analogWrite(DmotorL_PIN, pwmD);  
    analogWrite(DmotorR_PIN, 0);
  } else {
    analogWrite(DmotorL_PIN, 0);
    analogWrite(DmotorR_PIN, abs(pwmD));
  }
}

// ======================= Arm Function ====================== //
void controlArm() {
  static bool holding = false;
  static int last_target = 0;
  const int min_error = 5;
  const float Kp = 0.8;  // Pake proportional ajaa

  if (arm_target_position != last_target) {
    holding = true;
    last_target = arm_target_position;
  }

  if (holding) {
    int error = arm_target_position - encoderarm_count;

    // Cek jika sudah sampai di target
    if (abs(error) < min_error) {
      analogWrite(ARM_FORWARD_PIN, 0);
      analogWrite(ARM_BACKWARD_PIN, 0);
      holding = false;  // berhenti
      return;
    }
    // Menghitung PWM
    int pwm = Kp * error;
    pwm = constrain(pwm, -255, 255);
    analogWrite(ARM_FORWARD_PIN, pwm > 0 ? pwm : 0);
    analogWrite(ARM_BACKWARD_PIN, pwm < 0 ? -pwm : 0);

  } else {
    // motor mati jika tidak dalam mode holding
    analogWrite(ARM_FORWARD_PIN, 0);
    analogWrite(ARM_BACKWARD_PIN, 0);
  }
}
// Variabel PID Motor A
float KpA = 0.1,
      KiA = 0.00005,
      KdA = 0.0032;
float maxIntegralA = 1000.0; // BATAS MAKSIMUM UNTUK IA (Perlu Tuning)
float errorA = 0, PIDvalueA = 0;
float PA = 0, IA = 0, DA = 0;
float previousIA = 0, previousErrorA = 0;

// Variabel PID Motor B
float KpB = 0.100,
      KiB = 0.002,
      KdB = 0.00002;
float maxIntegralB = 1000.0; // BATAS MAKSIMUM UNTUK IB (Perlu Tuning)
float errorB = 0, PIDvalueB = 0;
float PB = 0, IB = 0, DB = 0;
float previousIB = 0, previousErrorB = 0;

// Variabel PID Motor C
float KpC = 0.1,
      KiC = 0.00002,
      KdC = 0.0016;
float maxIntegralC = 1000.0; // BATAS MAKSIMUM UNTUK IC (Perlu Tuning)
float errorC = 0, PIDvalueC = 0;
float PC = 0, IC = 0, DC = 0;
float previousIC = 0, previousErrorC = 0;

// Variabel PID Motor D
float KpD = 0.1,
      KiD = 0.00002,
      KdD = 0.00138;
float maxIntegralD = 1000.0; // BATAS MAKSIMUM UNTUK ID (Perlu Tuning)
float errorD = 0, PIDvalueD = 0;
float PD = 0, ID = 0, DD = 0;
float previousID = 0, previousErrorD = 0;

unsigned long lastTime = 0;


void pid() {
  unsigned long now = millis();
  double timeChange = (double)(now - lastTime);

  // ------------------ Motor A -------------------- //
  errorA = setpoint1 - rpmA; 

  if (setpoint1 == 0) {
    IA = 0; // Reset jika setpoint 0
  } else {
    IA = IA + (errorA * timeChange); // Akumulasi

    // --- PROTEKSI INTEGRAL CLAMPING ---
    if (IA > maxIntegralA) IA = maxIntegralA;
    else if (IA < -maxIntegralA) IA = -maxIntegralA;
    // ------------------------------------
  }

  DA = (errorA - previousErrorA) / timeChange;
  PIDvalueA = (KpA * errorA) + (KiA * IA) + (KdA * DA);

  PID_A = PIDvalueA;
  if (PID_A > 100) PID_A = 100;
  else if (PID_A < 0) PID_A = 0;

  SpeedA = PID_A;
  if (flagPID1) SpeedA *= -1;

  previousErrorA = errorA;

  // ------------------ Motor B -------------------- //
  errorB = setpoint2 - rpmB; 

  if (setpoint2 == 0) { 
    IB = 0;
  } else {
    IB = IB + (errorB * timeChange);

    // --- PROTEKSI INTEGRAL CLAMPING ---
    if (IB > maxIntegralB) IB = maxIntegralB;
    else if (IB < -maxIntegralB) IB = -maxIntegralB;
    // ------------------------------------
  }

  DB = (errorB - previousErrorB) / timeChange;
  PIDvalueB = (KpB * errorB) + (KiB * IB) + (KdB * DB);

  PID_B = PIDvalueB;
  if (PID_B > 100) PID_B = 100;
  else if (PID_B < 0) PID_B = 0;

  SpeedB = PID_B;
  if (flagPID2) SpeedB *= -1;

  previousErrorB = errorB;

  // ------------------ Motor C -------------------- //
  errorC = setpoint3 - rpmC;

  if (setpoint3 == 0) {
    IC = 0;
  } else {
    IC = IC + (errorC * timeChange);

    // --- PROTEKSI INTEGRAL CLAMPING ---
    if (IC > maxIntegralC) IC = maxIntegralC;
    else if (IC < -maxIntegralC) IC = -maxIntegralC;
    // ------------------------------------
  }

  DC = (errorC - previousErrorC) / timeChange;
  PIDvalueC = (KpC * errorC) + (KiC * IC) + (KdC * DC);

  PID_C = PIDvalueC;
  if (PID_C > 100) PID_C = 100;
  else if (PID_C < 0) PID_C = 0;

  SpeedC = PID_C;
  if (flagPID3) SpeedC *= -1;

  previousErrorC = errorC;

  // ------------------ Motor D -------------------- //
  errorD = setpoint4 - rpmD;

  if (setpoint4 == 0) {
    ID = 0;
  } else {
    ID = ID + (errorD * timeChange);

    // --- PROTEKSI INTEGRAL CLAMPING ---
    if (ID > maxIntegralD) ID = maxIntegralD;
    else if (ID < -maxIntegralD) ID = -maxIntegralD;
    // ------------------------------------
  }

  DD = (errorD - previousErrorD) / timeChange;
  PIDvalueD = (KpD * errorD) + (KiD * ID) + (KdD * DD);

  PID_D = PIDvalueD;
  if (PID_D > 100) PID_D = 100;
  else if (PID_D < 0) PID_D = 0;

  SpeedD = PID_D;
  if (flagPID4) SpeedD *= -1;

  previousErrorD = errorD;

// --------- reset interval --------- //
  lastTime = now;
}
void initial_setup() {
  // inisiasi Serial dan I2C dan analogfreq ada di main

  init_Motor();
  init_Encoder();
  init_Suction();
  init_Interrupt();
  init_Lidar();
  init_Timer();
  init_Compass();
  init_limitSwitch(); // baru nih
}

void init_limitSwitch() {
  pinMode(LimitSwitch_1, INPUT_PULLUP);
  pinMode(LimitSwitch_2, INPUT_PULLUP);
  pinMode(LimitSwitch_3, INPUT_PULLUP);
  pinMode(BUTTON, INPUT_PULLUP);
}

void init_Timer() {
// Kalau pake STM32F4 tidak perlu diubah
#if defined(TIM5)
  TIM_TypeDef *Instance = TIM5;
#else
  TIM_TypeDef *Instance = TIM5;
#endif
  HardwareTimer *MyTim = new HardwareTimer(Instance);
  MyTim->setOverflow(10, HERTZ_FORMAT);  // 10 Hz | 1/t = 10hz, t = 100 ms
  MyTim->attachInterrupt(updateoverflow);
  MyTim->resume();
}

void init_Lidar() {
  for (uint8_t i = 0; i < 2; i++) {  // for loop dari 0 sampai jumlah lidar (4)
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }
  for (uint8_t i = 0; i < 2; i++) {
    pinMode(xshutPins[i], INPUT);
    delay(10);
    lidar[i].setTimeout(500);
    if (!lidar[i].init()) {
      Serial.print("Gagal Membaca Sensor ");
      Serial.println(i);
    }
    lidar[i].setAddress(0x2A + i);
    lidar[i].startContinuous(50);
  }
}

void init_Encoder() {
  // --------- Inisiasi Encoder Motor ------------
  pinMode(AmotorENC_PIN, INPUT_PULLUP);
  pinMode(BmotorENC_PIN, INPUT_PULLUP);
  pinMode(CmotorENC_PIN, INPUT_PULLUP);
  pinMode(DmotorENC_PIN, INPUT_PULLUP);

  // ------- Inisiasi Encoder Rotary -------------
  pinMode(ODOMETRY_PIN_1A, INPUT_PULLUP);
  pinMode(ODOMETRY_PIN_1B, INPUT_PULLUP);
  pinMode(ODOMETRY_PIN_2A, INPUT_PULLUP);
  pinMode(ODOMETRY_PIN_2B, INPUT_PULLUP);
  pinMode(ODOMETRY_PIN_3A, INPUT_PULLUP);
  pinMode(ODOMETRY_PIN_3B, INPUT_PULLUP);

  // ------- Inisiasi Encoder Arm ----------------
  pinMode(ARM_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ARM_ENCODER_B_PIN, INPUT_PULLUP);
}

void init_Suction() {
  pinMode(SUCTION_PIN, OUTPUT);
  digitalWrite(SUCTION_PIN, LOW);
}

void init_Motor() {
  pinMode(AmotorR_PIN, OUTPUT);
  pinMode(AmotorL_PIN, OUTPUT);
  pinMode(BmotorR_PIN, OUTPUT);
  pinMode(BmotorL_PIN, OUTPUT);
  pinMode(CmotorR_PIN, OUTPUT);
  pinMode(CmotorL_PIN, OUTPUT);
  pinMode(DmotorR_PIN, OUTPUT);
  pinMode(DmotorL_PIN, OUTPUT);
  pinMode(ENABLE_MOTOR_PIN, OUTPUT);

  // pinMode(ARM_FORWARD_PIN, OUTPUT);
  // pinMode(ARM_BACKWARD_PIN, OUTPUT);
  // pinMode(ARM_LIMIT, INPUT);
}

void init_Interrupt() {
  attachInterrupt(digitalPinToInterrupt(AmotorENC_PIN), ISR_encoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(BmotorENC_PIN), ISR_encoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(CmotorENC_PIN), ISR_encoder3, RISING);
  attachInterrupt(digitalPinToInterrupt(DmotorENC_PIN), ISR_encoder4, RISING);

  attachInterrupt(digitalPinToInterrupt(ODOMETRY_PIN_1A), encA, RISING);
  attachInterrupt(digitalPinToInterrupt(ODOMETRY_PIN_2A), encB, RISING);
  attachInterrupt(digitalPinToInterrupt(ODOMETRY_PIN_3A), encC, RISING);

  attachInterrupt(digitalPinToInterrupt(ARM_ENCODER_A_PIN), ISR_encoderARM, RISING);
}

void init_Compass() {
  if (!bno.begin()) {
    // Serial.print("[i] Kompasnya ga kebaca elektrik!");
    while (1) {
    Serial.print("[i] Kompasnya ga kebaca elektrik!");
    }
  }

  delay(1000);  // kalau ganggu nanti hapus aja
}
void calculate_position() {
  pos_x = Odometry1 / 10;
  pos_y = Odometry2 / 10;
}

void read_compass() {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  float absoluteHeading = orientationData.orientation.x;
  
  heading = absoluteHeading - headingOffset;
  
  if (heading > 180) {
    heading -= 360;
  } else if (heading < -180) {
    heading += 360;
  }
}
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
  { "GO", handle_compass },
  { "RES",handle_reset },
  {"R", handle_kntl},
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
    x = parsedVx;
    y = parsedVy;
    z = parsedVw;

    // Debugging: Print the parsed values
    Serial.print("Parsed velocities - vx: ");
    Serial.print(x);
    Serial.print(", vy: ");
    Serial.print(y);
    Serial.print(", vw: ");
    Serial.println(z);

    // holonomic(x, y, z);
  } else {
    Serial.println("Invalid command format. Use: M vx vy vw");
  }
}

// ------------- Command S - Toggle Suction ------------ //
void handle_suction(char *args) {

  suction_state = !suction_state;

  if (suction_state) {
    digitalWrite(SUCTION_PIN, HIGH);
    Serial.println("1");
  } else {
    digitalWrite(SUCTION_PIN, LOW);
    Serial.println("0");
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
  // Siapkan buffer untuk menampung seluruh string data
  char data_buffer[100]; 

  // Format seluruh data ke dalam satu string menggunakan sprintf
  sprintf(data_buffer, "%ld %ld %ld %d %d %d %d %.2f %.2f %.2f",
          Odometry1,
          Odometry2,
          Odometry3,
          (int)encoder1RPM,
          (int)encoder2RPM,
          (int)encoder3RPM,
          (int)encoder4RPM,
          pos_x,
          pos_y,
          heading);

  // Kirim seluruh string sekali jalan dengan newline di akhir
  Serial.println(data_buffer);
}

// ------- Command GO - Run Home to Conveyor + print value initial compass ---- //
void handle_compass(char *args) {
  robotState = 1;
  Serial.println(heading);
}

// ------ Command RES - Reset Value for trial again ----------------------- //
void handle_reset(char *args) {
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
  // reset state robot
  robotState = 0;
  // reset compass
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  headingOffset = orientationData.orientation.x;

  Serial.println("SUDAH RESET!");
}

void handle_kntl(char *args) {
  robotState = 1;
}

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
  } else {
    if (robotState != 0) HomeToConveyor();
  }

  // Serial.println(heading);
  holonomic(x, y, -z);
  // 1 = 12 | 2 = 2
}