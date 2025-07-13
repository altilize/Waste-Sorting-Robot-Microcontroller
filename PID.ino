float KpA = 0.1,
      KiA = 0.05,
      KdA = 0.055;
float errorA = 0, PIDvalueA = 0;
float PA = 0, IA = 0, DA = 0;
float previousIA = 0, previousErrorA = 0;

float KpB = 0.1,
      KiB = 0.05,
      KdB = 0.055;
float errorB = 0, PIDvalueB = 0;
float PB = 0, IB = 0, DB = 0;
float previousIB = 0, previousErrorB = 0;

float KpC = 0.1,
      KiC = 0.05,
      KdC = 0.055;
float errorC = 0, PIDvalueC = 0;
float PC = 0, IC = 0, DC = 0;
float previousIC = 0, previousErrorC = 0;

float KpD = 0.1,
      KiD = 0.05,
      KdD = 0.055;
float errorD = 0, PIDvalueD = 0;
float PD = 0, ID = 0, DD = 0;
float previousID = 0, previousErrorD = 0;


void pid() {
  // ---------   MOTOR A ------------
  errorA = setpoint1 - rpmA;

  PA = errorA;
  IA = IA + previousIA;
  DA = errorA - previousErrorA;

  PIDvalueA = (KpA * PA) + (KiA * IA) + (KdA * DA);

  PID_A += PIDvalueA;
  if (PID_A > 255) PID_A = 255;
  else if (PID_A < 0) PID_A = 0;

  SpeedA = PID_A;
  if (flagPID1) SpeedA *= -1;

  previousErrorA = errorA;
  previousIA = IA;

  // ---------   MOTOR B ------------
  errorB = setpoint2 - rpmB;

  PB = errorB;
  IB = IB + previousIB;
  DB = errorB - previousErrorB;

  PIDvalueB = (KpB * PB) + (KiB * IB) + (KdB * DB);

  PID_B += PIDvalueB;
  if (PID_B > 255) PID_B = 255;
  else if (PID_B < 0) PID_B = 0;

  SpeedB = PID_B;
  if (flagPID2) SpeedB *= -1;

  previousErrorB = errorB;
  previousIB = IB;

  // ---------   MOTOR C ------------
  errorC = setpoint3 - rpmC;

  PC = errorC;
  IC = IC + previousIC;
  DC = errorC - previousErrorC;

  PIDvalueC = (KpC * PC) + (KiC * IC) + (KdC * DC);

  PID_C += PIDvalueC;
  if (PID_C > 255) PID_C = 255;
  else if (PID_C < 0) PID_C = 0;

  SpeedC = PID_C;
  if (flagPID3) SpeedC *= -1;

  previousErrorC = errorC;
  previousIC = IC;


  // ---------   MOTOR D ------------
  errorD = setpoint4 - rpmD;

  PD = errorD;
  ID = ID + previousID;
  DD = errorD - previousErrorD;

  PIDvalueD = (KpD * PD) + (KiD * ID) + (KdD * DD);

  PID_D += PIDvalueD;
  if (PID_D > 255) PID_D = 255;
  else if (PID_D < 0) PID_D = 0;

  SpeedD = PID_D;
  if (flagPID4) SpeedD *= -1;

  previousErrorD = errorD;
  previousID = ID;
}
