// ============ Motor ==================== //
#define AmotorL PA0
#define AmotorR PA2
#define BmotorR PB1
#define BmotorL PA7
#define CmotorL PA1
#define CmotorR PA3
#define DmotorL PA6
#define DmotorR PB0

// ========= Encoder Motor ============== //
#define AmotorENC PC7
#define BmotorENC PD13
#define CmotorENC PD12
#define DmotorENC PC6
volatile long encoderMotor1 = 0, encoderMotor2 = 0, encoderMotor3 = 0, encoderMotor4 = 0;
volatile long encoder1RPM = 0, encoder2RPM = 0, encoder3RPM = 0, encoder4RPM = 0;

// ----------- PID ----------
float setpoint1 = 0, setpoint2 = 0, setpoint3 = 0, setpoint4 = 0;
int rpmA = 0, rpmB = 0, rpmC = 0, rpmD = 0;
bool flagPID1 = false, flagPID2 = false, flagPID3 = false, flagPID4 = false;
float PID_A = 0, PID_B = 0, PID_C = 0, PID_D = 0;
float SpeedA = 0, SpeedB = 0, SpeedC = 0, SpeedD = 0;

// ============ Odometry =====================
#define Encoder1A PB10
#define Encoder1B PB12
#define Encoder2A PB11
#define Encoder2B PB13
#define Encoder3A PB15
#define Encoder3B PD9
volatile long Odometry1 = 0, Odometry2 = 0, Odometry3 = 0;

#define Enable PB2