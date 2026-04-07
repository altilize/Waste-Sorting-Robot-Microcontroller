# Dokumentasi Teknis — Firmware R2C BIMA

> **Authored by:** Moses Jaguar (622023006)
> **Platform:** STM32F407VETx — STM32Duino Framework
> **Terakhir diperbarui:** 7 April 2026

---

## 1. Gambaran Umum Sistem

Sistem ini merupakan **firmware mikrokontroler** untuk robot pemilah sampah berbasis roda **holonomic (omni-wheel) 4 motor**. Mikrokontroler STM32F407VETx bertindak sebagai *low-level controller* yang menangani:

- **Pergerakan robot** — Kinematika holonomic dengan kontrol PID per-motor
- **Navigasi otonom** — Menggunakan sensor Lidar, kompas (BNO055), dan line follower
- **Manipulasi objek** — Lengan robot (arm) dan mekanisme suction (vakum) untuk mengambil sampah
- **Mekanisme leadscrew** — Mengontrol pergerakan vertikal (naik/turun)
- **Komunikasi** — Menerima perintah dari komputer/Raspberry Pi melalui USB Serial

```
┌─────────────────────────────────────────────────────────────────┐
│                    ARSITEKTUR SISTEM                            │
│                                                                 │
│   ┌─────────────┐     USB Serial      ┌──────────────────┐     │
│   │   Laptop     │ ◄═══════════════► │   STM32F407VETx  │     │
│   │  (High-Level │    115200 baud      │  (Low-Level      │     │
│   │   Controller)│                     │   Controller)    │     │
│   └─────────────┘                     └────────┬─────────┘     │
│                                                 │               │
│                    ┌────────────────────────────┤               │
│                    │          │         │        │               │
│              ┌─────┴──┐ ┌────┴───┐ ┌───┴──┐ ┌──┴────┐         │
│              │ 4x     │ │ 2x     │ │ Arm  │ │ Line  │         │
│              │ Motor  │ │ Lidar  │ │ +    │ │Follower│         │
│              │ Omni   │ │VL53L1X │ │Suct. │ │ (MUX) │         │
│              └────────┘ └────────┘ └──────┘ └───────┘         │
│                                                                 │
│              ┌────────┐ ┌────────┐ ┌──────────────┐            │
│              │ 3x     │ │ BNO055 │ │  Leadscrew   │            │
│              │ Rotary │ │Compass │ │  Motor       │            │
│              │ Encoder│ │ (I2C)  │ │  + Encoder   │            │
│              └────────┘ └────────┘ └──────────────┘            │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. Arsitektur Perangkat Keras

### 2.1 Mikrokontroler

| Item | Spesifikasi |
|------|-------------|
| **MCU** | STM32F407VETx (ARM Cortex-M4, 168 MHz) |
| **Flash** | 512 KB |
| **SRAM** | 192 KB |
| **Framework** | STM32Duino (Arduino-compatible) |
| **Upload Method** | SWD (ST-Link) |
| **USB Support** | CDC Generic Serial (Supersede U(S)ART) |

### 2.2 Komponen Utama

| Komponen | Jumlah | Keterangan |
|----------|--------|------------|
| Motor DC + Encoder | 4 | Roda omni-wheel, PPR × Rasio = 134 |
| Rotary Encoder (Odometri) | 3 | Quadrature encoder untuk posisi X/Y |
| Lidar VL53L1X | 2 | Time-of-Flight, I2C, jarak depan kiri & kanan |
| BNO055 | 1 | IMU 9-DOF, digunakan sebagai kompas digital |
| Multiplexer 16-Channel | 1 | Untuk membaca 14 sensor line follower |
| Arm Motor + Encoder | 1 | Lengan robot dengan quadrature encoder |
| Leadscrew Motor + Encoder | 1 | Mekanisme naik/turun |
| Suction Pump | 1 | Vakum untuk mengambil objek |
| Limit Switch | 3 | Referensi posisi (homing) |
| Tombol (Button) | 1 | Emergency stop / reset manual |

---

## 3. Setup Development Environment

### 3.1 Prasyarat

- Arduino IDE atau PlatformIO
- STM32Duino core terinstal
- ST-Link programmer

### 3.2 Konfigurasi Board

```
Board Name      : STM32F407Vetx
Upload Method   : SWD
USB Support     : CDC Generic Serial Supersede U(S)ART
U(S)ART Support : Enabled Generic Serial
```

> 📎 Panduan setup lengkap: [STM32Duino on Ubuntu](https://github.com/altilize/STM32Duino-on-Ubuntu)

### 3.3 Mengatasi Port Tidak Terbaca

Jika port serial tidak terdeteksi setelah mencolokkan board:

```bash
# 1. Cek port yang tersedia
ls -l /dev/ttyACM*

# 2. Berikan permission
sudo chmod 666 /dev/ttyACM0    # sesuaikan nama port

# 3. Tambahkan user ke group dialout
sudo usermod -a -G dialout $USER

# 4. Refresh group (opsional, tanpa logout)
newgrp dialout
```

---

## 4. Struktur File Proyek

```
Waste-Sorting-Robot-Microcontroller/
│
├── Waste-Sorting-Robot-Microcontroller.ino   ← Deklarasi variabel global & pin
├── main.ino                                  ← setup() dan loop() — entry point
├── init.ino                                  ← Fungsi-fungsi inisialisasi hardware
├── Commands.ino                              ← Parser & handler command serial
├── Motion.ino                                ← Kinematika holonomic, arm, leadscrew
├── PID.ino                                   ← Kontrol PID untuk 4 motor
├── Interrupt.ino                             ← ISR encoder motor, rotary, arm, leadscrew
├── Lidar_LF.ino                              ← Pembacaan lidar, line follower, navigasi
├── Gyrodometry.ino                           ← Kalkulasi posisi & pembacaan kompas
├── asd.ino                                   ← File backup/arsip (seluruhnya di-comment)
└── DOKUMENTASI.md                            ← Dokumen ini
```

### Hubungan Antar File

```
main.ino ─── setup() ──── initial_setup() ─── [init.ino]
    │
    └─── loop()
           ├── calculate_position()  ←── [Gyrodometry.ino]
           ├── read_compass()        ←── [Gyrodometry.ino]
           ├── Lidar_Read()          ←── [Lidar_LF.ino]
           ├── read_command()        ←── [Commands.ino]
           ├── holonomic(x, y, -z)   ←── [Motion.ino]
           ├── update_leadscrew()    ←── [Motion.ino]
           └── controlArm()          ←── [Motion.ino]

Timer Interrupt (10 Hz):
    updateoverflow()  ←── [Interrupt.ino]
        ├── Hitung RPM dari encoder counter
        └── pid()     ←── [PID.ino]
```

---

## 5. Pinout & Wiring Map

### 5.1 Motor DC (4 Motor Omni-Wheel)

| Motor | Pin L (CW) | Pin R (CCW) | Pin Encoder |
|-------|-----------|------------|-------------|
| **A** | `PA0` | `PA2` | `PA5` |
| **B** | `PA1` | `PA3` | `PC6` |
| **C** | `PA6` | `PB0` | `PC7` |
| **D** | `PA7` | `PB1` | `PC4` |
| **Enable** | — | — | `PB2` |

> ⚠️ Motor C dan D memiliki catatan di source bahwa pin PA6↔PB0 dan PA7↔PB1 pernah ditukar.

### 5.2 Rotary Encoder (Odometri)

| Encoder | Pin A | Pin B |
|---------|-------|-------|
| **1** (sumbu X) | `PB12` | `PB10` |
| **2** (sumbu Y) | `PB11` | `PB13` |
| **3** | `PD9` | `PB15` |

### 5.3 Arm (Lengan Robot)

| Fungsi | Pin |
|--------|-----|
| Motor Forward | `PA10` |
| Motor Backward | `PE14` |
| Encoder A | `PC8` |
| Encoder B | `PA8` |
| Limit Switch (Home) | `PE3` |

### 5.4 Leadscrew

| Fungsi | Pin |
|--------|-----|
| Motor IN1 | `PE9` |
| Motor IN2 | `PE11` |
| Encoder A | `PC9` |
| Encoder B | `PA15` |

### 5.5 Sensor & Periph Lainnya

| Komponen | Pin |
|----------|-----|
| **Suction Pump** | `PD2` |
| **Lidar 1 XSHUT** | `PB5` |
| **Lidar 2 XSHUT** | `PB3` |
| **I2C SDA** | `PB7` |
| **I2C SCL** | `PB6` |
| **MUX Selektor 0** | `PC0` |
| **MUX Selektor 1** | `PC1` |
| **MUX Selektor 2** | `PC2` |
| **MUX Selektor 3** | `PC3` |
| **MUX Input (Analog)** | `PA4` |
| **Limit Switch 1** | `PE3` |
| **Limit Switch 2** | `PE5` |
| **Limit Switch 3** | `PE1` |
| **Button (Emergency)** | `PE6` |

---

## 6. Modul Software — Penjelasan Detail

### 6.1 Entry Point & Main Loop

**File:** `main.ino`

```cpp
void setup() {
    Serial.begin(115200);     // Inisialisasi serial USB
    Wire.setSDA(PB7);         // Konfigurasi I2C
    Wire.setSCL(PB6);
    Wire.begin();
    initial_setup();          // Inisialisasi semua hardware
}
```

**Loop utama** berjalan secara kontinyu dan melakukan:

```cpp
void loop() {
    calculate_position();     // Update posisi dari odometri
    read_compass();           // Baca heading dari BNO055
    Lidar_Read();             // Baca jarak dari 2 sensor lidar
    read_command();           // Proses perintah serial masuk

    if (digitalRead(BUTTON) == HIGH) {
        // EMERGENCY STOP — reset semua variabel
    } else {
        holonomic(x, y, -z);  // Eksekusi gerakan holonomic
        update_leadscrew();    // Update kontrol leadscrew
        controlArm();          // Update kontrol arm
    }
}
```

> **Catatan:** Variabel `z` di-invert (`-z`) saat dipanggil di `holonomic()`. Ini menyesuaikan konvensi arah rotasi dengan hardware.

#### Emergency Stop (Tombol)

Saat tombol `PE6` ditekan (HIGH), sistem langsung:
- Set kecepatan x, y, z = 0
- Reset semua variabel PID (SpeedA/B/C/D, PID_A/B/C/D)
- Reset odometri dan posisi
- Matikan enable motor (LOW)

---

### 6.2 Inisialisasi Perangkat Keras

**File:** `init.ino`

Fungsi `initial_setup()` memanggil seluruh inisialisasi secara berurutan:

```cpp
void initial_setup() {
    init_Motor();        // Pin motor → OUTPUT
    init_Encoder();      // Pin encoder → INPUT_PULLUP
    init_Suction();      // Pin suction → OUTPUT, default LOW (mati)
    init_Interrupt();    // Attach semua ISR ke pin interrupt
    init_Lidar();        // Init 2x VL53L1X via I2C, set alamat unik
    init_Timer();        // Hardware Timer TIM5, 10 Hz
    init_Compass();      // Init BNO055, blocking jika gagal
    init_limitSwitch();  // Pin limit switch → INPUT_PULLUP
    init_LF();           // Pin multiplexer → OUTPUT/INPUT
    init_LS();           // Pin leadscrew → OUTPUT, encoder → INPUT_PULLUP
}
```

#### Detail: Inisialisasi Lidar

Kedua sensor VL53L1X terhubung pada bus I2C yang sama. Untuk menghindari konflik alamat:

1. Semua pin XSHUT di-set LOW (mematikan semua sensor)
2. Satu per satu, pin XSHUT di-set INPUT (menyalakan sensor)
3. Setiap sensor diberi alamat I2C unik: `0x2A`, `0x2B`
4. Diset mode *continuous* dengan interval 50 ms

#### Detail: Hardware Timer

Timer TIM5 digunakan untuk membuat **interrupt berkala tiap 100 ms (10 Hz)**. Callback-nya adalah `updateoverflow()` yang menghitung RPM dan menjalankan PID.

```
Frekuensi Timer = 10 Hz → Interval = 100 ms
```

#### Detail: Kompas BNO055

Inisialisasi BNO055 bersifat **blocking** — jika sensor tidak terdeteksi, program akan hang di infinite loop sambil mencetak pesan error. Ini desain yang disengaja agar robot tidak bergerak tanpa kompas.

---

### 6.3 Kinematika Holonomic (Omni-Wheel)

**File:** `Motion.ino` — fungsi `holonomic()`

Robot menggunakan **4 roda omni-wheel** yang memungkinkan pergerakan ke segala arah tanpa harus berputar terlebih dahulu.

#### Matriks Kinematika

```
SpeedA = (-0.35 × vx) + ( 0.35 × vy) + (0.25 × vz)
SpeedB = (-0.35 × vx) + (-0.35 × vy) + (0.25 × vz)
SpeedC = ( 0.35 × vx) + (-0.35 × vy) + (0.25 × vz)
SpeedD = ( 0.35 × vx) + ( 0.35 × vy) + (0.25 × vz)
```

| Parameter | Keterangan |
|-----------|-----------|
| `vx` | Kecepatan translasi sumbu X (kiri/kanan) |
| `vy` | Kecepatan translasi sumbu Y (maju/mundur) |
| `vz` | Kecepatan rotasi (putar CW/CCW) |
| `0.35` | Koefisien translasi (terkait geometri robot) |
| `0.25` | Koefisien rotasi (terkait jarak roda ke pusat) |

#### Alur Proses

```
Input (vx, vy, vz)
      │
      ▼
Hitung kecepatan per-motor (matriks kinematika)
      │
      ▼
Kalikan ×10 → setpoint PID (setpoint1..4)
      │
      ▼
Tentukan flag arah (flagPID1..4)
      │
      ▼
Ambil nilai absolut setpoint
      │
      ▼
motorauto() → Tulis PWM ke pin motor driver
```

#### Fungsi `motorauto()`

Menerjemahkan nilai `SpeedA..D` (output PID) menjadi sinyal PWM ke motor driver:

- Jika `pwm > 0` → motor berputar ke satu arah (pin L = pwm, pin R = 0)
- Jika `pwm < 0` → motor berputar arah sebaliknya (pin L = 0, pin R = |pwm|)
- `ENABLE_MOTOR_PIN` di-set HIGH sebelum menulis PWM

---

### 6.4 Kontrol PID Motor

**File:** `PID.ino`

Setiap motor memiliki kontroler **PID (Proportional-Integral-Derivative)** independen. PID dijalankan oleh timer interrupt pada frekuensi **10 Hz**.

#### Parameter PID

| Motor | Kp | Ki | Kd | Max Integral |
|-------|------|---------|---------|-------------|
| **A** | 0.1 | 0.00005 | 0.0032 | 1000.0 |
| **B** | 0.1 | 0.00002 | 0.00002 | 1000.0 |
| **C** | 0.1 | 0.00002 | 0.0016 | 1000.0 |
| **D** | 0.1 | 0.00002 | 0.00138 | 1000.0 |

> ⚠️ Ki dan Kd berbeda tiap motor karena karakteristik fisik motor yang tidak identik. Nilai-nilai ini memerlukan tuning empiris.

#### Algoritma PID (per Motor)

```
1. Hitung error = setpoint - RPM_aktual
2. Jika setpoint == 0 → reset integral (I = 0)
3. Jika tidak:
     I = I + (error × Δt)
     Clamping: -maxIntegral ≤ I ≤ maxIntegral
4. D = (error - error_sebelumnya) / Δt
5. PID_output = (Kp × error) + (Ki × I) + (Kd × D)
6. Clamping output: 0 ≤ PID ≤ 100
7. Aplikasikan flag arah (negate jika flagPID = true)
```

#### Integral Clamping (Anti-Windup)

Mekanisme anti-windup mencegah akumulasi integral berlebihan saat error besar dalam waktu lama. Ini menghindari overshoot saat robot baru mulai bergerak atau saat ada hambatan.

```
if (I > maxIntegral)  I = maxIntegral;
if (I < -maxIntegral) I = -maxIntegral;
```

---

### 6.5 Interrupt Service Routines (ISR)

**File:** `Interrupt.ino`

#### ISR Encoder Motor (Rising Edge Only)

```cpp
void ISR_encoder1() { encoderMotor1++; }   // Motor A
void ISR_encoder2() { encoderMotor2++; }   // Motor B
void ISR_encoder3() { encoderMotor3++; }   // Motor C
void ISR_encoder4() { encoderMotor4++; }   // Motor D
```

Hanya menghitung jumlah tick. Arah sudah diketahui dari flag PID.

#### ISR Encoder Arm & Leadscrew (Quadrature)

Menggunakan **quadrature decoding** untuk mengetahui arah rotasi:

```cpp
void ISR_encoderARM() {
    if (digitalRead(A) == digitalRead(B))
        encoderarm_count--;
    else
        encoderarm_count++;
}
```

Logika yang sama digunakan untuk `handle_ls_encoder()`.

#### ISR Rotary Encoder (Odometri)

Tiga encoder rotary untuk menghitung posisi robot menggunakan quadrature:

```cpp
void encA() {
    (digitalRead(Pin_A) == digitalRead(Pin_B)) ? Odometry1-- : Odometry1++;
}
```

#### Timer Interrupt — `updateoverflow()` (10 Hz)

Dipanggil setiap 100 ms oleh hardware timer TIM5:

```
1. Simpan counter encoder ke variabel RPM (encoder1RPM..4RPM)
2. Reset counter encoder ke 0
3. Hitung RPM aktual:
     RPM = (encoderTick × 60 × 10) / 134
     
     Penjelasan:
       × 60    = konversi ke per-menit
       × 10    = konversi dari 100ms ke 1 detik
       ÷ 134   = PPR × rasio gear motor
       
4. Jalankan pid()
```

---

### 6.6 Odometri & Kompas

**File:** `Gyrodometry.ino`

#### Kalkulasi Posisi

```cpp
void calculate_position() {
    pos_x = Odometry1 / 10;
    pos_y = Odometry2 / 10;
}
```

Posisi dihitung langsung dari tick encoder rotary yang dibagi 10 untuk konversi ke satuan yang lebih berarti (approx. mm → cm atau sejenisnya, tergantung kalibrasi diameter roda encoder).

#### Pembacaan Kompas

```cpp
void read_compass() {
    // Baca heading absolut dari BNO055
    float absoluteHeading = orientationData.orientation.x;
    
    // Kurangi dengan offset (di-set saat reset)
    heading = absoluteHeading - headingOffset;
    
    // Normalisasi ke range [-180°, 180°]
    if (heading > 180) heading -= 360;
    else if (heading < -180) heading += 360;
}
```

**`headingOffset`** di-set saat perintah `RES` (reset) dijalankan, sehingga heading saat reset menjadi 0°.

---

### 6.7 Sensor Lidar (VL53L1X)

**File:** `Lidar_LF.ino` — fungsi `Lidar_Read()`, `Debug_Lidar()`

```cpp
void Lidar_Read() {
    readLidar[0] = lidar[0].read();   // Sensor kiri
    readLidar[1] = lidar[1].read();   // Sensor kanan
}
```

| Sensor | Index | Pin XSHUT | Alamat I2C | Posisi |
|--------|-------|-----------|------------|--------|
| Lidar 0 | `readLidar[0]` | `PB5` | `0x2A` | Kiri |
| Lidar 1 | `readLidar[1]` | `PB3` | `0x2B` | Kanan |

Mode operasi: **Continuous**, interval 50 ms, timeout 500 ms.

---

### 6.8 Line Follower (Multiplexer)

**File:** `Lidar_LF.ino` — fungsi `LF_Read()`, `LF_WeightedAverage()`

#### Arsitektur Sensor

14 sensor infrared terhubung melalui **multiplexer 16-channel** (CD4067 atau sejenisnya). Hanya 1 pin analog (`PA4`) yang digunakan, dipilih via 4 pin selektor.

```
Selektor (PC0-PC3) → MUX → PA4 (analogRead)
```

#### Pembacaan Sensor (`LF_Read`)

Membaca 14 channel MUX secara berurutan:

```cpp
for (ch = 0..13) {
    MUX_Select(ch);
    delayMicroseconds(50);         // Settling time
    LF_Vertikal[ch] = analogRead(MUX_INPUT);
}
```

#### Weighted Average (`LF_WeightedAverage`)

Menghitung posisi garis relatif terhadap pusat sensor menggunakan bobot:

```
Bobot sensor: [30, 20, 10, 0, -10, -20, -30]
                ←kiri        tengah       kanan→
```

- Sensor yang membaca nilai ≥ 650 dianggap aktif (mendeteksi garis)
- Output = rata-rata bobot sensor aktif
- Jika output > 0 → garis di kiri, robot harus koreksi ke kiri
- Jika output < 0 → garis di kanan, robot harus koreksi ke kanan
- Jika output = 0 → garis di tengah, atau tidak ada garis terdeteksi

---

### 6.9 Kontrol Arm (Lengan Robot)

**File:** `Motion.ino` — fungsi `controlArm()`

Lengan robot menggunakan **P-Controller** (kontrol proporsional) dengan fitur:

1. **Deadband** — Toleransi error 5 tick encoder. Jika sudah dalam range ini, motor dimatikan.
2. **Minimum Power** — PWM minimum 40 agar motor tidak stuck saat mendekati target.
3. **Clamping** — PWM dibatasi [-255, 255].

```
error = target - posisi_aktual

Jika |error| < 5 → STOP (motor mati)
Jika tidak:
    pwm = 0.8 × error
    pwm = constrain(pwm, -255, 255)
    
    // Minimum power
    if (0 < pwm < 40) → pwm = 40
    if (-40 < pwm < 0) → pwm = -40
```

#### Homing Arm

Perintah `AH` menggerakkan arm ke posisi home (limit switch):

```
1. Gerakkan arm maju dengan PWM = 80
2. Tunggu sampai limit switch tertekan (LOW)
3. Stop motor
4. Reset encoder arm ke 0
5. Set arm_homed = true
```

Timeout: 5 detik. Jika limit switch tidak tertekan dalam 5 detik, homing dibatalkan.

---

### 6.10 Kontrol Leadscrew

**File:** `Motion.ino` — fungsi `update_leadscrew()`, `set_leadscrew_motor()`

Leadscrew digunakan untuk pergerakan **vertikal** (menaikkan/menurunkan platform). Menggunakan **P-Controller** sederhana.

#### Parameter

| Parameter | Nilai | Keterangan |
|-----------|-------|------------|
| `kp_leadscrew` | 1.0 | Konstanta proporsional |
| `ls_tolerance` | 10 | Deadband encoder tick |
| `ls_speed_pwm` | 100 | Kecepatan default (belum dipakai di logic P-control) |
| Minimum PWM | 60 | Agar motor tidak berdengung |

#### Logika Kontrol

```
Jika tidak aktif → return (tidak lakukan apa-apa)

error = target_pos - current_pos

Jika |error| ≤ toleransi:
    STOP motor (brake mode)
    Set ls_active = false
    return

speed = error × Kp
speed = constrain(speed, -255, 255)

// Minimum power
if (0 < speed < 60) → speed = 60
if (-60 < speed < 0) → speed = -60

set_leadscrew_motor(speed)
```

#### Mode Motor Leadscrew

| Speed | Aksi |
|-------|------|
| `> 0` | Gerak naik: IN1 = speed, IN2 = 0 |
| `< 0` | Gerak turun: IN1 = 0, IN2 = |speed| |
| `= 0` | Brake: IN1 = 255, IN2 = 255 |

---

### 6.11 Suction (Vakum)

**File:** `Commands.ino` — fungsi `handle_suction()`

Suction beroperasi sebagai **toggle**:

```cpp
suction_state = !suction_state;
if (suction_state) → HIGH (nyala)
else              → LOW  (mati)
```

Setiap kali perintah `S` dikirim, state suction akan berubah (nyala ↔ mati).

---

### 6.12 Algoritma Navigasi — HomeToConveyor

**File:** `Lidar_LF.ino` — fungsi `HomeToConveyor()`

Algoritma navigasi otonom menggunakan **state machine** dengan 6 state:

```
┌───────────┐    pos_y≥600    ┌───────────┐   heading≈-88°   ┌───────────┐
│  State 1  │ ─────────────► │  State 2  │ ───────────────► │  State 3  │
│ Maju Lurus│                │ Berputar  │                  │ Maju ke   │
│ y=150     │                │ ke -88°   │                  │ Halangan  │
└───────────┘                └───────────┘                  └─────┬─────┘
                                                                  │
                                                        lidar ≤ 150cm
                                                                  │
                                                            ┌─────▼─────┐
┌───────────┐    pos seimbang ┌───────────┐                 │  State 4  │
│  State 5  │ ◄────────────── │           │ ◄───────────── │ Seimbang  │
│ Geser     │                │  State 4  │    |diff|≤5     │ Koreksi   │
│ Kanan     │                │ Balancing │                 │ Rotasi    │
└─────┬─────┘                └───────────┘                 └───────────┘
      │
      │ pos_x ≥ 200
      ▼
┌───────────┐
│  State 6  │ ◄──── Bolak-balik terus antara State 5 ↔ 6
│ Geser     │
│ Kiri      │
└───────────┘
```

#### Detail Tiap State

| State | Aksi | Kondisi Transisi |
|-------|------|-----------------|
| **1** | Maju lurus (y=150) | `pos_y ≥ 600` → State 2 |
| **2** | Berputar ke heading -88° (P-control, Kp=0.7) | `|error| ≤ 1°` → Reset odometri → State 3 |
| **3** | Maju lurus (y=100) menuju halangan | `lidar[0] ≤ 150` ATAU `lidar[1] ≤ 150` → State 4 |
| **4** | Seimbangkan posisi tegak lurus halangan (koreksi rotasi) | `|diff lidar| ≤ 5` → State 5 |
| **5** | Geser kanan (x=100) + koreksi jarak & rotasi real-time | `pos_x ≥ 200` → State 6 |
| **6** | Geser kiri (x=-100) + koreksi jarak & rotasi real-time | `pos_x ≤ 0` → State 5 |

#### Koreksi Real-Time (State 5 & 6)

Saat bergeser, robot secara simultan menjaga:
- **Jarak ke halangan (Y)** — P-control dengan `Kp = 1.5`, target 150 cm
- **Tegak lurus (Z)** — P-control dengan `Kp = 0.8`, berdasarkan selisih lidar kiri-kanan
- Koreksi Y dan Z dibatasi ±40

---

## 7. Protokol Komunikasi Serial

### 7.1 Konfigurasi

```
Baudrate  : 115200
Format    : 8N1 (8 data bits, no parity, 1 stop bit)
Delimiter : Newline (\n)
```

### 7.2 Daftar Perintah

| Perintah | Format | Keterangan | Contoh |
|----------|--------|------------|--------|
| `M` | `M <vx> <vy> <vw>` | Gerakkan robot (holonomic) | `M 100 0 0` → geser kanan |
| `S` | `S` | Toggle suction ON/OFF | `S` |
| `A` | `A <pos>` | Set posisi target arm | `A 500` |
| `AH` | `AH` | Homing arm (ke limit switch) | `AH` |
| `L` | `L <pos>` | Set posisi target leadscrew | `L 5000` |
| `RLS` | `RLS` | Reset encoder leadscrew ke 0 | `RLS` |
| `R` | `R` | Baca jarak lidar kiri & kanan | `R` |
| `RAW_LF` | `RAW_LF` | Baca nilai analog line follower | `RAW_LF` |
| `DIG_LF` | `DIG_LF` | Baca nilai digital line follower | `DIG_LF` |
| `?` | `?` | Query semua data sensor | `?` |
| `GO` | `GO` | Mulai navigasi otonom (Home→Conveyor) | `GO` |
| `RES` | `RES` | Reset semua variabel & heading offset | `RES` |

### 7.3 Format Respons

#### Perintah `?` (Query Data Sensor)

```
D <pos_x> <pos_y> <heading> <lidar_L> <lidar_R> <leadscrew_pos> <arm_pos>
```

**Contoh output:**
```
D 150 320 -45.5 200 195 1500 800
```

| Field | Index | Keterangan |
|-------|-------|------------|
| `D` | 0 | Header/marker |
| `pos_x` | 1 | Posisi X dari odometri |
| `pos_y` | 2 | Posisi Y dari odometri |
| `heading` | 3 | Heading kompas (derajat, -180 s/d 180) |
| `lidar_L` | 4 | Jarak lidar kiri (mm) |
| `lidar_R` | 5 | Jarak lidar kanan (mm) |
| `leadscrew_pos` | 6 | Posisi encoder leadscrew |
| `arm_pos` | 7 | Posisi encoder arm |

#### Perintah `R` (Lidar)

```
L:<jarak_kiri> R:<jarak_kanan>
```

#### Perintah `S` (Suction)

```
1   → Suction dinyalakan
0   → Suction dimatikan
```

### 7.4 Internal — Command Parser

Menggunakan **lookup table** berbasis struct:

```cpp
typedef struct {
    const char *cmd;
    void (*handler)(char *args);
} Command;

Command commands[] = {
    { "A",      handle_arm },
    { "AH",     handle_home_arm },
    { "M",      handle_movement },
    // ... dan seterusnya
};
```

Parser menggunakan `strtok()` untuk memisahkan command dan argumen, kemudian melakukan linear search pada tabel command.

---

## 8. Diagram Alur Sistem

### 8.1 Alur Eksekusi Utama

```
                    ┌──────────────────┐
                    │   POWER ON       │
                    └────────┬─────────┘
                             │
                    ┌────────▼─────────┐
                    │    setup()       │
                    │  Serial 115200   │
                    │  I2C Init        │
                    │  initial_setup() │
                    └────────┬─────────┘
                             │
              ┌──────────────▼──────────────┐
              │          loop()             │◄──────────────┐
              │                             │               │
              │  1. calculate_position()    │               │
              │  2. read_compass()          │               │
              │  3. Lidar_Read()            │               │
              │  4. read_command()          │               │
              │                             │               │
              │  ┌──── BUTTON? ────┐        │               │
              │  │                 │        │               │
              │  ▼ YES             ▼ NO     │               │
              │ EMERGENCY        NORMAL     │               │
              │ STOP          ┌─────────┐   │               │
              │ (reset all)   │holonomic│   │               │
              │               │leadscrew│   │               │
              │               │arm      │   │               │
              │               └─────────┘   │               │
              └─────────────────────────────┘               │
                             │                              │
                             └──────────────────────────────┘
```

### 8.2 Alur PID + Timer

```
    Timer TIM5 (10 Hz)
         │
         ▼
  updateoverflow()
         │
    ┌────┴────┐
    │         │
    ▼         ▼
 Capture    Reset
 encoder    encoder        RPM = (tick × 600) / 134
 ke RPM     counter
    │
    ▼
  pid()
    │
    ├── Motor A: error = setpoint1 - rpmA → PID → SpeedA
    ├── Motor B: error = setpoint2 - rpmB → PID → SpeedB
    ├── Motor C: error = setpoint3 - rpmC → PID → SpeedC
    └── Motor D: error = setpoint4 - rpmD → PID → SpeedD
```

---

## 9. Tabel Konstanta & Parameter Tuning

### 9.1 Kinematika

| Konstanta | Nilai | Keterangan |
|-----------|-------|------------|
| Koefisien translasi | 0.35 | Faktor kecepatan translasi per motor |
| Koefisien rotasi | 0.25 | Faktor kecepatan rotasi per motor |
| Scale factor setpoint | ×10 | Konversi kecepatan → setpoint PID |

### 9.2 PID Motor

| Parameter | Motor A | Motor B | Motor C | Motor D |
|-----------|---------|---------|---------|---------|
| **Kp** | 0.1 | 0.1 | 0.1 | 0.1 |
| **Ki** | 5×10⁻⁵ | 2×10⁻⁵ | 2×10⁻⁵ | 2×10⁻⁵ |
| **Kd** | 3.2×10⁻³ | 2×10⁻⁵ | 1.6×10⁻³ | 1.38×10⁻³ |
| **Max Integral** | 1000 | 1000 | 1000 | 1000 |
| **Output Range** | 0–100 | 0–100 | 0–100 | 0–100 |

### 9.3 Arm

| Parameter | Nilai |
|-----------|-------|
| Kp | 0.8 |
| Deadband | 5 tick |
| Min PWM | 40 |
| Max PWM | 255 |
| Homing PWM | 80 |
| Homing Timeout | 5000 ms |

### 9.4 Leadscrew

| Parameter | Nilai |
|-----------|-------|
| Kp | 1.0 |
| Toleransi (Deadband) | 10 tick |
| Min PWM | 60 |
| Max PWM | 255 |
| Brake Mode | IN1=255, IN2=255 |

### 9.5 Navigasi (HomeToConveyor)

| Parameter | Nilai |
|-----------|-------|
| Target Jarak Halangan | 150 cm |
| Toleransi Seimbang | 5 cm |
| Toleransi Jarak | 5 cm |
| Kp Distance | 1.5 |
| Kp Balance | 0.8 |
| Max Koreksi Speed | 40 |
| Target Heading (State 2) | -88° |
| Kp Heading | 0.7 |
| Toleransi Heading | 1° |
| Min Rotasi Speed | 70 |

### 9.6 Encoder

| Parameter | Nilai |
|-----------|-------|
| PPR × Rasio Motor | 134 |
| Timer Interval | 100 ms (10 Hz) |
| RPM Formula | `tick × 60 × 10 / 134` |

### 9.7 Line Follower

| Parameter | Nilai |
|-----------|-------|
| Threshold Aktif | ≥ 650 (analog) |
| Threshold Debug | ≥ 400 (analog) |
| Jumlah Sensor | 14 channel (0–13) |
| Settling Time | 50 µs per channel |
| Bobot Sensor | [30, 20, 10, 0, -10, -20, -30] |

---

## 10. Troubleshooting

### 10.1 Port Serial Tidak Terdeteksi

```bash
# Cek apakah device terdeteksi
ls -l /dev/ttyACM*

# Set permission
sudo chmod 666 /dev/ttyACM0

# Tambah user ke group
sudo usermod -a -G dialout $USER
newgrp dialout
```

### 10.2 Kompas Tidak Terbaca

**Gejala:** Program hang saat boot, mencetak `[i] Kompasnya ga kebaca elektrik!` berulang.

**Solusi:**
- Cek koneksi I2C (SDA = PB7, SCL = PB6)
- Pastikan alamat BNO055 sesuai (default `0x28`)
- Cek suplai daya 3.3V ke BNO055

### 10.3 Motor Bergerak Tidak Sesuai

**Langkah debugging:**
1. Kirim `?` untuk cek data sensor
2. Kirim `M 100 0 0` — robot harus geser kanan
3. Kirim `M 0 100 0` — robot harus maju
4. Jika terbalik, cek wiring motor di pin header
5. Jika berputar tak terkendali, cek parameter PID (lihat §9.2)

### 10.4 Lidar Gagal Init

**Gejala:** Mencetak `Gagal Membaca Sensor X`.

**Solusi:**
- Cek koneksi I2C
- Cek pin XSHUT (PB5, PB3)
- Pastikan suplai daya 2.8V–3.3V
- Cek apakah ada short di bus I2C

### 10.5 PID Overshoot / Osilasi

- Turunkan `Kp` secara bertahap
- Naikkan `Kd` sedikit untuk damping
- Pastikan `maxIntegral` tidak terlalu besar
- Cek apakah encoder memberikan bacaan yang benar (kirim perintah debug)

### 10.6 Robot Tidak Merespons Perintah

- Pastikan baudrate serial = 115200
- Perintah harus diakhiri newline (`\n`)
- Cek apakah BUTTON (PE6) tidak dalam keadaan HIGH (emergency stop)
- Perintah case-sensitive: gunakan huruf KAPITAL (`M`, bukan `m`)

---

> Dokumentasi ini ditulis oleh **Moses Jaguar Ersaptiarso**

---


