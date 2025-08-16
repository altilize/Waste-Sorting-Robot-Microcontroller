float KpA = 0.0002,
      KiA = 0.00008,
      KdA = 0.00002;
float errorA = 0, PIDvalueA = 0;
float PA = 0, IA = 0, DA = 0;
float previousIA = 0, previousErrorA = 0;

float KpB = 0.0002,
      KiB = 0.00001,
      KdB = 0.0;
float errorB = 0, PIDvalueB = 0;
float PB = 0, IB = 0, DB = 0;
float previousIB = 0, previousErrorB = 0;

float KpC = 0.0002,
      KiC = 0.00001,
      KdC = 0.0;
float errorC = 0, PIDvalueC = 0;
float PC = 0, IC = 0, DC = 0;
float previousIC = 0, previousErrorC = 0;

float KpD = 0.0002,
      KiD = 0.00001,
      KdD = 0.0;
float errorD = 0, PIDvalueD = 0;
float PD = 0, ID = 0, DD = 0;
float previousID = 0, previousErrorD = 0;


void pid() {
// ------------------ Motor A -------------------- //
  errorA = setpoint1 - rpmA;

  PA = errorA;
  IA = IA + previousIA;
  DA = errorA - previousErrorA;

  PIDvalueA = (KpA * PA) + (KiA * IA) + (KdA * DA);

  PID_A += PIDvalueA;
  if (PID_A > 255) PID_A = 255;
  else if (PID_A < -255) PID_A = -255;

  SpeedA = PID_A;
  if (flagPID1) SpeedA *= -1;

  previousErrorA = errorA;
  previousIA = IA;

// ------------------ Motor B -------------------- //
  errorB = setpoint2 - rpmB;

  PB = errorB;
  IB = IB + previousIB;
  DB = errorB - previousErrorB;

  PIDvalueB = (KpB * PB) + (KiB * IB) + (KdB * DB);

  PID_B += PIDvalueB;
  if (PID_B > 255) PID_B = 255;
  else if (PID_B < -255) PID_B = -255;

  SpeedB = PID_B;
  if (flagPID2) SpeedB *= -1;

  previousErrorB = errorB;
  previousIB = IB;

// ------------------ Motor C -------------------- //
  errorC = setpoint3 - rpmC;

  PC = errorC;
  IC = IC + previousIC;
  DC = errorC - previousErrorC;

  PIDvalueC = (KpC * PC) + (KiC * IC) + (KdC * DC);

  PID_C += PIDvalueC;
  if (PID_C > 255) PID_C = 255;
  else if (PID_C < -255) PID_C = -255;

  SpeedC = PID_C;
  if (flagPID3) SpeedC *= -1;

  previousErrorC = errorC;
  previousIC = IC;

// ------------------ Motor D -------------------- //
  errorD = setpoint4 - rpmD;

  PD = errorD;
  ID = ID + previousID;
  DD = errorD - previousErrorD;

  PIDvalueD = (KpD * PD) + (KiD * ID) + (KdD * DD);

  PID_D += PIDvalueD;
  if (PID_D > 255) PID_D = 255;
  else if (PID_D < -255) PID_D = -255;

  SpeedD = PID_D;
  if (flagPID4) SpeedD *= -1;

  previousErrorD = errorD;
  previousID = ID;
}
