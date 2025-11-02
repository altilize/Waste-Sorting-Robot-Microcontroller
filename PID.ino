float KpA = 0.1,
      KiA = 0.0,
      KdA = 0.0;
float errorA = 0, PIDvalueA = 0;
float PA = 0, IA = 0, DA = 0;
float previousIA = 0, previousErrorA = 0;

float KpB = 0.1,
      KiB = 0.00001,
      KdB = 0.0;
float errorB = 0, PIDvalueB = 0;
float PB = 0, IB = 0, DB = 0;
float previousIB = 0, previousErrorB = 0;

float KpC = 0.1,
      KiC = 0.0,
      KdC = 0.0;
float errorC = 0, PIDvalueC = 0;
float PC = 0, IC = 0, DC = 0;
float previousIC = 0, previousErrorC = 0;

float KpD = 0.1,
      KiD = 0.0,
      KdD = 0.0001;
float errorD = 0, PIDvalueD = 0;
float PD = 0, ID = 0, DD = 0;
float previousID = 0, previousErrorD = 0;

unsigned long lastTime = 0;


void newPID() {
  unsigned long now = millis();
  double timeChange = (double)(now - lastTime);

  // ------------------ Motor A -------------------- //
  errorA = setpoint1 - rpmA; 

  if (setpoint1 == 0) {
    IA = 0;
  } else {
    IA = IA + (errorA * timeChange); 
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
    IB = IB + (errorB * timeChange);  // 'errSum'
  }

  DB = (errorB - previousErrorB) / timeChange;  // D ('dErr')

  PIDvalueB = (KpB * errorB) + (KiB * IB) + (KdB * DB);  // Output

  PID_B = PIDvalueB;  // Clamping
  if (PID_B > 100) PID_B = 100;
  else if (PID_B < 0) PID_B = 0;

  SpeedB = PID_B;
  if (flagPID2) SpeedB *= -1;

  previousErrorB = errorB;  // Simpan 'lastErr'

  // ------------------ Motor C -------------------- //
  errorC = setpoint3 - rpmC;  // P

  if (setpoint3 == 0) {  // I (dengan anti-windup)
    IC = 0;
  } else {
    IC = IC + (errorC * timeChange);  // 'errSum'
  }

  DC = (errorC - previousErrorC) / timeChange;  // D ('dErr')

  PIDvalueC = (KpC * errorC) + (KiC * IC) + (KdC * DC);  // Output

  PID_C = PIDvalueC;  // Clamping
  if (PID_C > 100) PID_C = 100;
  else if (PID_C < 0) PID_C = 0;

  SpeedC = PID_C;
  if (flagPID3) SpeedC *= -1;

  previousErrorC = errorC;  // Simpan 'lastErr'

  // ------------------ Motor D -------------------- //
  errorD = setpoint4 - rpmD;  // P

  if (setpoint4 == 0) {  // I (dengan anti-windup)
    ID = 0;
  } else {
    ID = ID + (errorD * timeChange);  // 'errSum'
  }

  DD = (errorD - previousErrorD) / timeChange;  // D ('dErr')

  PIDvalueD = (KpD * errorD) + (KiD * ID) + (KdD * DD);  // Output

  PID_D = PIDvalueD;  // Clamping
  if (PID_D > 100) PID_D = 100;
  else if (PID_D < 0) PID_D = 0;

  SpeedD = PID_D;
  if (flagPID4) SpeedD *= -1;

  previousErrorD = errorD;  // Simpan 'lastErr'

  /* Ingat waktu ini untuk perhitungan 'timeChange' berikutnya */
  lastTime = now;
}

void pid() {
  // ------------------ Motor A -------------------- //
  errorA = setpoint1 - rpmA;

  PA = errorA;

  // *** FIX ANTI-WINDUP ***
  // Jika setpoint adalah 0, paksa Integral ke 0 (reset memori).
  // Jika tidak, baru akumulasi error.
  if (setpoint1 == 0) {
    IA = 0;
  } else {
    IA = IA + errorA;
  }

  DA = errorA - previousErrorA;
  PIDvalueA = (KpA * PA) + (KiA * IA) + (KdA * DA);

  PID_A = PIDvalueA;
  if (PID_A > 100) PID_A = 100;
  else if (PID_A < 0) PID_A = 0;

  SpeedA = PID_A;
  if (flagPID1) SpeedA *= -1;

  previousErrorA = errorA;

  // ------------------ Motor B -------------------- //
  errorB = setpoint2 - rpmB;

  PB = errorB;

  // *** FIX ANTI-WINDUP ***
  if (setpoint2 == 0) {
    IB = 0;
  } else {
    IB = IB + errorB;
  }

  DB = errorB - previousErrorB;
  PIDvalueB = (KpB * PB) + (KiB * IB) + (KdB * DB);

  PID_B = PIDvalueB;
  if (PID_B > 100) PID_B = 100;
  else if (PID_B < 0) PID_B = 0;

  SpeedB = PID_B;
  if (flagPID2) SpeedB *= -1;

  previousErrorB = errorB;

  // ------------------ Motor C -------------------- //
  errorC = setpoint3 - rpmC;

  PC = errorC;

  // *** FIX ANTI-WINDUP ***
  if (setpoint3 == 0) {
    IC = 0;
  } else {
    IC = IC + errorC;
  }

  DC = errorC - previousErrorC;
  PIDvalueC = (KpC * PC) + (KiC * IC) + (KdC * DC);

  PID_C = PIDvalueC;
  if (PID_C > 100) PID_C = 100;
  else if (PID_C < 0) PID_C = 0;

  SpeedC = PID_C;
  if (flagPID3) SpeedC *= -1;

  previousErrorC = errorC;

  // ------------------ Motor D -------------------- //
  errorD = setpoint4 - rpmD;

  PD = errorD;

  // *** FIX ANTI-WINDUP ***
  if (setpoint4 == 0) {
    ID = 0;
  } else {
    ID = ID + errorD;
  }

  DD = errorD - previousErrorD;
  PIDvalueD = (KpD * PD) + (KiD * ID) + (KdD * DD);

  PID_D = PIDvalueD;
  if (PID_D > 100) PID_D = 100;
  else if (PID_D < 0) PID_D = 0;

  SpeedD = PID_D;
  if (flagPID4) SpeedD *= -1;

  previousErrorD = errorD;
}