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