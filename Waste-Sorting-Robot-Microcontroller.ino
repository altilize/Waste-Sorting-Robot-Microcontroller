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

// ============ Motor ==================== //
#define AmotorL PA0
#define AmotorR PA2
#define BmotorR PA7 // PB1
#define BmotorL PB1 // PA7
#define CmotorL PA1
#define CmotorR PA3
#define DmotorL PB0 // PA6
#define DmotorR PA6 // PB0
#define Enable PB2

// ============== Encoder Motor ==============
#define AmotorENC PC7
#define BmotorENC PC4 // PD13 ubah ke PC4
#define CmotorENC PA5 // PD12 ubah ke PA5
#define DmotorENC PC6
volatile int encoderMotor1 = 0, encoderMotor2 = 0, encoderMotor3 = 0, encoderMotor4 = 0;
volatile int encoder1RPM = 0, encoder2RPM = 0, encoder3RPM = 0, encoder4RPM = 0;
 
// ----------- PID ----------
float setpoint1 = 0, setpoint2 = 0, setpoint3 = 0, setpoint4 = 0;
int rpmA = 0, rpmB = 0, rpmC = 0, rpmD = 0;
bool flagPID1 = false, flagPID2 = false, flagPID3 = false, flagPID4 = false;
float PID_A = 0, PID_B = 0, PID_C = 0, PID_D = 0;
float SpeedA = 0, SpeedB = 0, SpeedC = 0, SpeedD = 0;

// ============ Odometry ===================== //
#define Encoder1A PB10
#define Encoder1B PB12
#define Encoder2A PB11
#define Encoder2B PB13
#define Encoder3A PB15
#define Encoder3B PD9
volatile int Odometry1 = 0, Odometry2 = 0, Odometry3 = 0;

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
volatile int LF_Vertikal[8];
float weights[] = { 0.3, 0.2, 0.1, 0, -0.1, -0.2, -0.3 };


// ============== Variabel Lidar =================== //
#include <VL53L1X.h>

const uint8_t xshutPins[2] = { PB5, PB3 };
VL53L1X lidar[2];
double readLidar[2];