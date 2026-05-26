# 🤖 ESP32 Bluetooth Robot Arm & RC Car

## Dokumentasi Teknikal — Proyek Embedded System

---

> **Platform:** ESP32 DevKit V1 | **Komunikasi:** Bluetooth Classic (Dabble)  
> **Versi Firmware:** 1.0 | **Status:** Stable / Functional  
> **Author:** Chiekal / Krenova Team

---

## 📋 Daftar Isi

1. [Overview Proyek](#1-overview-proyek)
2. [Arsitektur Sistem](#2-arsitektur-sistem)
3. [Dependensi & Library](#3-dependensi--library)
4. [Program 1: Robot Arm (4-DOF)](#4-program-1-robot-arm-4-dof)
5. [Program 2: RC Car (4-Wheel)](#5-program-2-rc-car-4-wheel)
6. [Wiring & Koneksi Hardware](#6-wiring--koneksi-hardware)
7. [Konfigurasi & Flashing](#7-konfigurasi--flashing)
8. [Analisis Komparatif Kedua Program](#8-analisis-komparatif-kedua-program)
9. [Known Issues & Catatan Teknis](#9-known-issues--catatan-teknis)
10. [To-Do & Pengembangan Lanjutan](#10-to-do--pengembangan-lanjutan)

---

## 1. Overview Proyek

Proyek ini terdiri dari **dua firmware terpisah** untuk platform ESP32 yang dikendalikan secara nirkabel menggunakan aplikasi **Dabble** via Bluetooth Classic. Keduanya menggunakan modul GamePad Dabble sebagai antarmuka kontrol utama.

| Firmware  | File                                                              | Fungsi                                           |
| --------- | ----------------------------------------------------------------- | ------------------------------------------------ |
| Robot Arm | [ESP32_Bluetooth_RobotArm.ino](code/ESP32_Bluetooth_RobotArm.ino) | Kontrol 4 servo (Base, Shoulder, Elbow, Gripper) |
| RC Car    | [ESP32_Bluetooth_Car.ino](ESP32_Bluetooth_Car.ino)                | Kontrol 4 DC motor via L298N driver              |

**Perbedaan fundamental** antara keduanya: Robot Arm mengimplementasikan *smooth interpolation* berbasis target, sementara RC Car menggunakan kontrol on/off sederhana (bang-bang control).

---

## 2. Arsitektur Sistem

```
┌─────────────────────────────────────────────────┐
│                  SMARTPHONE                     │
│   ┌─────────────────────────────────────────┐   │
│   │           Dabble App (GamePad)          │   │
│   └──────────────────┬──────────────────────┘   │
└──────────────────────│──────────────────────────┘
                       │ Bluetooth Classic (SPP)
                       ▼
┌─────────────────────────────────────────────────┐
│                   ESP32                         │
│  ┌────────────────────────────────────────┐     │
│  │  Dabble.processInput()  — BT Handler   │     │
│  ├────────────────────────────────────────┤     │ 
│  │  handleInput()          — Input Parser │     │
│  ├────────────────────────────────────────┤     │
│  │  smoothUpdate() / rotateMotor()        │     │
│  └──────────────┬─────────────────────────┘     │
└─────────────────│───────────────────────────────┘
                  │
       ┌──────────┴──────────┐
       ▼                     ▼
 [Robot Arm]            [RC Car]
 4x Servo Motor         L298N + 4x DC Motor
```

---

## 3. Dependensi & Library

### Library yang Dibutuhkan

| Library       | Source                                                                            | Fungsi                             |
| ------------- | --------------------------------------------------------------------------------- | ---------------------------------- |
| `DabbleESP32` | Arduino Library Manager or [here](https://docs.arduino.cc/libraries/dabbleesp32/) | Bluetooth GamePad interface        |
| `ESP32Servo`  | Arduino Library Manager or [here](https://docs.arduino.cc/libraries/esp32servo/)  | PWM servo control (Robot Arm only) |

### Instalasi via Arduino IDE

```
Tools → Manage Libraries → Search:
  ├── "DabbleESP32" by STEMpedia
  └── "ESP32Servo" by Kevin Harrington
```

### Atau

```
Sketch → Include Library → Add .zip Library
  ├── "DabbleESP32.zip"
  └── "ESP32Servo.zip" 
```

### Preprocessor Defines (Wajib)

```cpp
// Harus dideklarasikan SEBELUM include DabbleESP32.h
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>
```

> ⚠️ **Penting:** Kedua `#define` di atas wajib ada. `CUSTOM_SETTINGS` memberi tahu Dabble untuk tidak menggunakan konfigurasi default, sedangkan `INCLUDE_GAMEPAD_MODULE` mengaktifkan parsing paket GamePad secara spesifik. Tanpa ini, `GamePad.*` tidak akan terkompilasi.

---

## 4. Program 1: Robot Arm (4-DOF)

### 4.1 Deskripsi

Firmware untuk mengendalikan robot arm dengan 4 Degrees of Freedom menggunakan 4 servo motor. Fitur utamanya adalah **smooth movement** berbasis interpolasi target — servo tidak langsung melompat ke posisi baru, melainkan bergerak bertahap setiap tick timer.

### 4.2 Koneksi Pin

```cpp
#define SERVO_BASE_PIN      25   // GPIO25 — Base: rotasi kiri/kanan
#define SERVO_SHOULDER_PIN  26   // GPIO26 — Shoulder: naik/turun
#define SERVO_ELBOW_PIN     27   // GPIO27 — Elbow: extend/retract
#define SERVO_GRIPPER_PIN   14   // GPIO14 — Gripper: buka/tutup
```

> **Catatan:** GPIO 25, 26, 27, dan 14 dipilih karena merupakan GPIO "aman" di ESP32 DevKit V1 — tidak dipakai secara internal (bootloader, flash SPI, dll.).

### 4.3 Konstanta Kontrol

```cpp
#define SERVO_MIN        0       // Batas bawah sudut servo
#define SERVO_MAX        180     // Batas atas sudut servo
#define GRIPPER_OPEN     30      // Posisi capit terbuka penuh
#define GRIPPER_CLOSE    150     // Posisi capit tertutup penuh

#define INPUT_STEP       3       // Δ target per tombol ditekan (derajat)
#define SMOOTH_STEP      1       // Δ posisi fisik per tick (derajat)
#define SERVO_INTERVAL   15      // Periode update servo (ms) ≈ 66 Hz
```

**Implikasi timing:**

- Dengan `SMOOTH_STEP = 1` dan `SERVO_INTERVAL = 15ms`, kecepatan gerak servo ≈ **66°/detik**
- Dengan `INPUT_STEP = 3`, setiap tombol ditekan menambah target **3°**; servo mengejar secara smooth

### 4.4 ⚡ Bagian Program Penting

#### A. State Variables — Dualitas Current/Target

```cpp
// ============================================================
//   KONSEP KUNCI: Setiap servo punya DUA nilai posisi
// ============================================================
int currentBase     = 90,  targetBase     = 90;  // Base
int currentShoulder = 90,  targetShoulder = 90;  // Shoulder
int currentElbow    = 90,  targetElbow    = 90;  // Elbow
int currentGripper  = 90,  targetGripper  = 90;  // Gripper
```

> `current*` = posisi fisik servo saat ini (yang ditulis ke PWM)  
> `target*` = posisi tujuan (diubah oleh tombol GamePad)

Inisialisasi di 90° (tengah) agar servo tidak meloncat saat boot.

---

#### B. `clamp()` & `stepToward()` — Fungsi Helper Kritis

```cpp
// Batasi nilai dalam rentang [lo, hi]
inline int clamp(int val, int lo, int hi) {
  return val < lo ? lo : (val > hi ? hi : val);
}

// Gerakkan 'current' satu langkah SMOOTH_STEP menuju 'target'
inline int stepToward(int current, int target, int step) {
  if (current < target) return min(current + step, target);
  if (current > target) return max(current - step, target);
  return current;
}
```

`clamp()` mencegah nilai servo melampaui batas mekanik. `stepToward()` adalah inti dari smooth movement — memindahkan posisi current satu langkah per tick tanpa overshoot.

---

#### C. `handleInput()` — Mapping Tombol ke Target

```cpp
void handleInput() {
  // Base — DPad Kiri/Kanan
  if (GamePad.isLeftPressed())   targetBase = clamp(targetBase - INPUT_STEP, SERVO_MIN, SERVO_MAX);
  if (GamePad.isRightPressed())  targetBase = clamp(targetBase + INPUT_STEP, SERVO_MIN, SERVO_MAX);

  // Shoulder — Segitiga naik / Cross turun
  if (GamePad.isTrianglePressed()) targetShoulder = clamp(targetShoulder + INPUT_STEP, SERVO_MIN, SERVO_MAX);
  if (GamePad.isCrossPressed())    targetShoulder = clamp(targetShoulder - INPUT_STEP, SERVO_MIN, SERVO_MAX);

  // Elbow — DPad Atas naik / DPad Bawah turun
  if (GamePad.isUpPressed())   targetElbow = clamp(targetElbow + INPUT_STEP, SERVO_MIN, SERVO_MAX);
  if (GamePad.isDownPressed()) targetElbow = clamp(targetElbow - INPUT_STEP, SERVO_MIN, SERVO_MAX);

  // Gripper — Square buka / Circle tutup
  if (GamePad.isSquarePressed()) targetGripper = clamp(targetGripper - INPUT_STEP, GRIPPER_OPEN,  GRIPPER_CLOSE);
  if (GamePad.isCirclePressed()) targetGripper = clamp(targetGripper + INPUT_STEP, GRIPPER_OPEN,  GRIPPER_CLOSE);
}
```

> **Catatan desain:** Gripper menggunakan batas `GRIPPER_OPEN`/`GRIPPER_CLOSE` (bukan `SERVO_MIN`/`SERVO_MAX`) karena range mekanik capit lebih sempit. Ini mencegah motor servo stall.

---

#### D. `smoothUpdate()` — Interpolasi & Write PWM

```cpp
void smoothUpdate() {
  currentBase     = stepToward(currentBase,     targetBase,     SMOOTH_STEP);
  currentShoulder = stepToward(currentShoulder, targetShoulder, SMOOTH_STEP);
  currentElbow    = stepToward(currentElbow,    targetElbow,    SMOOTH_STEP);
  currentGripper  = stepToward(currentGripper,  targetGripper,  SMOOTH_STEP);
  writeServos();  // Tulis semua posisi ke PWM
}
```

---

#### E. `setupServos()` — Inisialisasi Timer PWM

```cpp
void setupServos() {
  // Alokasi 4 hardware timer ESP32 untuk PWM servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Set frekuensi PWM 50 Hz (standar servo analog/digital)
  servoBase.setPeriodHertz(50);
  servoShoulder.setPeriodHertz(50);
  servoElbow.setPeriodHertz(50);
  servoGripper.setPeriodHertz(50);

  // Attach dengan pulse width 500–2400 µs
  // (range lebih lebar dari standar 1000–2000 µs untuk akurasi lebih baik)
  servoBase.attach(SERVO_BASE_PIN,      500, 2400);
  servoShoulder.attach(SERVO_SHOULDER_PIN, 500, 2400);
  servoElbow.attach(SERVO_ELBOW_PIN,    500, 2400);
  servoGripper.attach(SERVO_GRIPPER_PIN, 500, 2400);

  writeServos(); // Set posisi awal 90° sebelum motor bergerak
}
```

> **Kenapa 500–2400 µs?** Standar servo adalah 1000–2000 µs. Memperluas ke 500–2400 µs meningkatkan resolusi sudut dan memastikan servo mencapai 0° dan 180° penuh, namun berpotensi melewati batas mekanik pada servo murah. Sesuaikan jika servo stall di ujung range.

---

#### F. `loop()` — Non-Blocking Timer

```cpp
void loop() {
  Dabble.processInput();  // Proses paket BT yang masuk

  unsigned long now = millis();
  if (now - lastServoUpdate >= SERVO_INTERVAL) {
    lastServoUpdate = now;
    handleInput();    // 1. Baca tombol → update target
    smoothUpdate();   // 2. Geser current → target → write PWM
  }
}
```

> Pola `millis()` non-blocking ini krusial — `delay()` akan memblokir `Dabble.processInput()` dan menyebabkan BT buffer overflow/disconnect.

### 4.5 Mapping Tombol GamePad

| Tombol       | Joint    | Arah         |
| ------------ | -------- | ------------ |
| DPad ←       | Base     | Rotasi Kiri  |
| DPad →       | Base     | Rotasi Kanan |
| △ (Triangle) | Shoulder | Naik         |
| ✕ (Cross)    | Shoulder | Turun        |
| DPad ↑       | Elbow    | Extend       |
| DPad ↓       | Elbow    | Retract      |
| □ (Square)   | Gripper  | Buka         |
| ○ (Circle)   | Gripper  | Tutup        |

---

## 5. Program 2: RC Car (4-Wheel)

### 5.1 Deskripsi

Firmware untuk RC Car 4 roda dengan motor driver L298N. Menggunakan **PWM via LEDC** (hardware PWM bawaan ESP32) untuk kontrol kecepatan. Kontrol bersifat on/off penuh — tidak ada akselerasi bertahap.

### 5.2 Koneksi Pin

```cpp
// Motor Kanan
int enableRightMotor = 22;   // GPIO22 — EN_A (PWM speed)
int rightMotorPin1   = 16;   // GPIO16 — IN1
int rightMotorPin2   = 17;   // GPIO17 — IN2

// Motor Kiri
int enableLeftMotor  = 23;   // GPIO23 — EN_B (PWM speed)
int leftMotorPin1    = 18;   // GPIO18 — IN3
int leftMotorPin2    = 19;   // GPIO19 — IN4
```

### 5.3 Konfigurasi PWM

```cpp
#define MAX_MOTOR_SPEED   255    // Resolusi 8-bit (0–255)
const int PWMFreq       = 1000; // 1 kHz — cocok untuk motor DC brushed
const int PWMResolution = 8;    // 8-bit = 256 level kecepatan
const int rightMotorPWMSpeedChannel = 4;  // LEDC channel 4
const int leftMotorPWMSpeedChannel  = 5;  // LEDC channel 5
```

### 5.4 Bagian Program Penting

#### A. `rotateMotor()` — Logika Arah & Kecepatan

```cpp
void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {
  // Tentukan arah motor kanan berdasarkan TANDA nilai speed
  if (rightMotorSpeed < 0) {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);   // Mundur
  } else if (rightMotorSpeed > 0) {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);    // Maju
  } else {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);    // Brake (short-circuit brake)
  }

  // Logika yang sama untuk motor kiri...

  // Tulis magnitude (nilai absolut) ke LEDC PWM
  ledcWrite(rightMotorPWMSpeedChannel, abs(rightMotorSpeed));
  ledcWrite(leftMotorPWMSpeedChannel,  abs(leftMotorSpeed));
}
```

> **Desain penting:** Fungsi menerima nilai bertanda (signed) — negatif = mundur, positif = maju, nol = brake. `abs()` digunakan untuk memisahkan logika arah (pin IN1/IN2) dari logika kecepatan (LEDC PWM). Ini adalah pola yang bersih dan mudah di-extend.

---

#### B. `setUpPinModes()` — Inisialisasi LEDC

```cpp
void setUpPinModes() {
  // Set pin sebagai output
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  // ... (kiri juga)

  // Konfigurasi LEDC channel
  ledcSetup(rightMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  ledcSetup(leftMotorPWMSpeedChannel,  PWMFreq, PWMResolution);

  // Bind LEDC channel ke pin Enable L298N
  ledcAttachPin(enableRightMotor, rightMotorPWMSpeedChannel);
  ledcAttachPin(enableLeftMotor,  leftMotorPWMSpeedChannel);

  rotateMotor(0, 0); // Pastikan motor berhenti saat boot
}
```

> **Catatan:** ESP32 menggunakan **LEDC (LED Control)** peripheral untuk PWM general-purpose, bukan `analogWrite()` seperti Arduino biasa. `ledcSetup()` → `ledcAttachPin()` → `ledcWrite()` adalah urutan wajib.

---

#### C. `loop()` — Kontrol Arah Sederhana

```cpp
void loop() {
  int rightMotorSpeed = 0;
  int leftMotorSpeed  = 0;

  Dabble.processInput();

  if (GamePad.isUpPressed()) {
    rightMotorSpeed = MAX_MOTOR_SPEED;   // Maju: kedua motor maju
    leftMotorSpeed  = MAX_MOTOR_SPEED;
  }
  if (GamePad.isDownPressed()) {
    rightMotorSpeed = -MAX_MOTOR_SPEED;  // Mundur: kedua motor mundur
    leftMotorSpeed  = -MAX_MOTOR_SPEED;
  }
  if (GamePad.isLeftPressed()) {
    rightMotorSpeed =  MAX_MOTOR_SPEED;  // Belok kiri: kanan maju, kiri mundur
    leftMotorSpeed  = -MAX_MOTOR_SPEED;  // (pivot/spin turn)
  }
  if (GamePad.isRightPressed()) {
    rightMotorSpeed = -MAX_MOTOR_SPEED;  // Belok kanan: kanan mundur, kiri maju
    leftMotorSpeed  =  MAX_MOTOR_SPEED;
  }

  rotateMotor(rightMotorSpeed, leftMotorSpeed);
}
```

> **Tipe belok:** Menggunakan **differential/pivot turn** (satu motor maju, satu mundur), bukan gradual turn (satu motor lebih lambat). Ini membuat mobil berputar di tempat, responsif tapi terasa agresif. Bisa dimodifikasi untuk smooth turn.

### 5.5 Mapping Tombol GamePad

| Tombol | Aksi                | Motor Kanan | Motor Kiri |
| ------ | ------------------- | ----------- | ---------- |
| DPad ↑ | Maju                | +255        | +255       |
| DPad ↓ | Mundur              | -255        | -255       |
| DPad ← | Belok Kiri (Pivot)  | +255        | -255       |
| DPad → | Belok Kanan (Pivot) | -255        | +255       |
| (none) | Berhenti            | 0           | 0          |

---

## 6. Wiring & Koneksi Hardware

### 6.1 Robot Arm — Servo Wiring

![](img/Hand%20Wiring.png)

```
ESP32          Servo
─────────────────────────────
GPIO 25  ──── Signal (Base)
GPIO 26  ──── Signal (Shoulder)
GPIO 27  ──── Signal (Elbow)
GPIO 14  ──── Signal (Gripper)
3.3V/5V  ──── VCC semua servo  ← Gunakan catu daya eksternal!
GND      ──── GND semua servo
```

> ⚠️ **Peringatan daya:** Jangan menyuplai 4 servo langsung dari pin 3.3V/5V ESP32. Arus stall servo bisa mencapai 1–2A/unit. Gunakan catu daya eksternal 5V minimal 3A, dengan GND yang di-share ke ESP32.

Kode inisialisasi servo mengalokasikan **4 hardware timer** terpisah (Timer 0–3) karena setiap timer di ESP32 dapat di-share oleh beberapa channel PWM, namun untuk servo yang butuh timing presisi 50Hz, lebih aman 1 timer per servo:

```cpp
ESP32PWM::allocateTimer(0);  // Timer untuk Base
ESP32PWM::allocateTimer(1);  // Timer untuk Shoulder
ESP32PWM::allocateTimer(2);  // Timer untuk Elbow
ESP32PWM::allocateTimer(3);  // Timer untuk Gripper
```

### 6.2 RC Car — Motor Driver Wiring

![](img/WIRING%20DIAGRAM.jpg)

```
ESP32       L298N          Motor
──────────────────────────────────────────
GPIO 22 ── ENA          ── Right Motor Speed
GPIO 16 ── IN1 ─────────── Right Motor1 (+)
GPIO 17 ── IN2 ─────────── Right Motor1 (−)
GPIO 23 ── ENB          ── Left Motor Speed
GPIO 18 ── IN3 ─────────── Left Motor2 (+)
GPIO 19 ── IN4 ─────────── Left Motor2 (−)

L298N VCC ── 7–12V DC (eksternal)
L298N GND ── GND (shared dengan ESP32)
L298N 5V  ── VIN ESP32 (opsional, jika pakai onboard regulator L298N)
```

> Setiap sisi (kanan/kiri) menggunakan **2 motor DC paralel** — Right Motor1 + Right Motor2 berjalan bersama dari output L298N yang sama. Total 4 motor untuk traksi 4-wheel drive.

---

## 7. Konfigurasi & Flashing

### 7.1 Arduino IDE Setup

```
Board       : ESP32 Dev Module  (atau sesuai board yang dipakai)
Upload Speed: 115200 baud
CPU Freq    : 240 MHz (default)
Flash Freq  : 80 MHz
Partition   : Default 4MB with spiffs
```

### 7.2 Bluetooth Device Name

```cpp
// Robot Arm
Dabble.begin("RobotArm_ESP32");

// RC Car
Dabble.begin("MyBluetoothCar");
```

Nama ini yang muncul di Bluetooth pairing di smartphone. Ubah sesuai kebutuhan sebelum flash.

### 7.3 Aplikasi Dabble

1. Install **Dabble** dari Play Store / App Store
2. Buka app → Pilih **GamePad** modul
3. Koneksikan ke device BT sesuai nama di atas
4. Pastikan format GamePad **tidak** dalam mode analog joystick (gunakan mode D-Pad digital)

---

## 8. Analisis Komparatif Kedua Program

| Aspek               | Robot Arm                                                | RC Car                        |
| ------------------- | -------------------------------------------------------- | ----------------------------- |
| **Kontrol jenis**   | Target-based smooth interpolation                        | Bang-bang (on/off penuh)      |
| **Timer**           | `millis()` non-blocking (15ms interval)                  | Polling langsung di `loop()`  |
| **Output aktuator** | Servo PWM (ESP32Servo)                                   | DC Motor via LEDC + L298N     |
| **Frekuensi PWM**   | 50 Hz (servo standard)                                   | 1000 Hz (motor DC)            |
| **Respons input**   | Bertahap (smooth, bisa overshoot karena inertia mekanik) | Instan (hard stop/full speed) |
| **Safety limiter**  | `clamp()` pada semua servo                               | Tidak ada (selalu MAX_SPEED)  |
| **Serial debug**    | Tidak ada                                                | Ada (`Serial.print` arah)     |
| **BT device name**  | `RobotArm_ESP32`                                         | `MyBluetoothCar`              |

### Mengapa Pendekatan Berbeda?

Servo memerlukan smooth movement untuk mencegah **jerk/shock load** pada joint dan gearbox. Motor DC di car justru butuh **responsivitas penuh** untuk manuver cepat, Akselerasi bertahap justru terasa "lag". Desain ini sudah tepat untuk masing-masing use case.

---

## 9. Known Issues & Catatan Teknis

### ⚠️ Issue 1: Tidak Ada Akselerasi/Deselerasi pada RC Car

Program RC Car langsung memberikan `MAX_MOTOR_SPEED = 255` tanpa ramp-up. Pada motor dengan gear ratio rendah, ini bisa menyebabkan wheel slip dan lonjakan arus saat start.

**Workaround sementara:** Kurangi `MAX_MOTOR_SPEED` ke ~180 untuk start yang lebih halus.

---

### ⚠️ Issue 2: Tombol DPad Tumpang Tindih di Robot Arm

DPad Atas/Bawah dipakai untuk **Elbow** di Robot Arm, namun di RC Car dipakai untuk **Maju/Mundur**. Bila source code digabung atau confuse, pastikan firmware yang benar ter-flash.

---

### ⚠️ Issue 3: Tidak Ada Posisi Preset / Home Position

Tidak ada fungsi "return to home" atau preset posisi. Servo hanya diinisialisasi di 90° saat boot. Jika koneksi BT terputus saat lengan dalam posisi ekstrem, lengan tetap di posisi tersebut.

---

### ⚠️ Issue 4: RC Car — Serial Debug Blocking Potential

```cpp
Serial.print("up_pressed"); // Inside loop(), setiap iterasi
```

`Serial.print()` di dalam loop tanpa kondisi dapat menyebabkan buffer bottleneck pada baud rate rendah. Tidak kritis pada 115200 baud, tapi perlu dibersihkan untuk produksi.

---

### ℹ️ Catatan: Pulse Width Servo 500–2400 µs

Range ini lebih lebar dari standar industri (1000–2000 µs). Beberapa servo murah (SG90, MG996R klonan) bisa **stall atau overheat** di ujung range. Monitor suhu servo saat pertama kali testing.

---

## 10. To-Do & Pengembangan Lanjutan

### 🔴 Prioritas Tinggi

- [ ] **[Robot Arm] Tambah home position button**: Assign salah satu tombol (mis. Start/Select) untuk mengembalikan semua servo ke 90° secara smooth. Penting untuk keamanan operasional.

- [ ] **[RC Car] Implementasi PWM speed control bertahap**: Tambah variabel kecepatan yang bisa dikontrol (mis. tombol R1/R2 untuk throttle), bukan hanya full-on/full-off.

- [ ] **[RC Car] Hapus Serial.print debug** dari loop produksi, atau bungkus dengan flag:
  
  ```cpp
  #define DEBUG_MODE 1
  #if DEBUG_MODE
    Serial.print("up_pressed");
  #endif
  ```

- [ ] **Kombinasikan Program**: Ubah program agar hanya menggunakan 1 Dev Board ESP32 dengan cara menggabungkan program

### 🟡 Prioritas Menengah

- [ ] **[Robot Arm] Simpan & replay posisi**: Rekam sequence posisi servo ke array (EEPROM/SPIFFS), lalu playback otomatis. Berguna untuk demonstrasi atau pick-and-place berulang.

- [ ] **[Robot Arm] Feedback posisi via Serial/BT**: Kirim balik `currentBase`, `currentShoulder`, dst. ke app untuk ditampilkan. Dabble mendukung terminal/serial module.

- [ ] **[RC Car] Implementasi gradual/smooth turn**: Ganti pivot turn dengan pengurangan kecepatan satu sisi:
  
  ```cpp
  // Contoh smooth left turn
  if (GamePad.isLeftPressed()) {
    rightMotorSpeed = MAX_MOTOR_SPEED;
    leftMotorSpeed  = MAX_MOTOR_SPEED / 2;  // Kiri lebih lambat
  }
  ```

- [ ] **[Keduanya] Watchdog / Timeout koneksi BT**: Jika koneksi BT hilang, servo/motor harus berhenti otomatis. Implementasikan dengan `millis()` timestamp terakhir input valid.

### 🟢 Pengembangan Jangka Panjang

- [ ] **Inverse Kinematics (Robot Arm)** — Alih-alih kontrol per-joint manual, implementasikan IK solver sederhana agar user bisa input koordinat (X, Y, Z) target end-effector.

- [ ] **Sensor feedback (RC Car)** — Integrasikan ultrasonic sensor (HC-SR04) untuk obstacle avoidance otomatis.

- [ ] **OTA (Over-the-Air) Update** — Implementasikan Arduino OTA agar firmware bisa diperbarui via WiFi tanpa kabel USB.

- [ ] **PCB custom** — Rancang PCB dedicated untuk merapikan wiring, menambah proteksi reverse polarity, dan voltage regulator terpadu.

---

## Appendix — Referensi Cepat GPIO

### ESP32 — GPIO Safe for Output

| GPIO | Robot Arm      | RC Car             | Catatan                                     |
| ---- | -------------- | ------------------ | ------------------------------------------- |
| 14   | Servo Gripper  | —                  | Boot mode sensitive (hindari LOW saat boot) |
| 16   | —              | RightMotor IN1     | —                                           |
| 17   | —              | RightMotor IN2     | —                                           |
| 18   | —              | LeftMotor IN3      | —                                           |
| 19   | —              | LeftMotor IN4      | —                                           |
| 22   | —              | Enable Right (PWM) | —                                           |
| 23   | —              | Enable Left (PWM)  | —                                           |
| 25   | Servo Base     | —                  | DAC-capable                                 |
| 26   | Servo Shoulder | —                  | DAC-capable                                 |
| 27   | Servo Elbow    | —                  | —                                           |

### GPIO yang Harus Dihindari untuk Output

| GPIO  | Alasan                                      |
| ----- | ------------------------------------------- |
| 0     | Boot mode strapping pin                     |
| 2     | Onboard LED, boot mode                      |
| 6–11  | Terhubung ke SPI Flash internal             |
| 12    | Boot mode strapping, tegangan flash         |
| 34–39 | Input only, tidak ada pull-up/down internal |

---

*Dokumentasi ini dibuat berdasarkan analisis source code dan wiring diagram yang tersedia dalam repositori proyek.*
