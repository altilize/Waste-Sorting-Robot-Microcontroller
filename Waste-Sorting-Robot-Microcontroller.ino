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