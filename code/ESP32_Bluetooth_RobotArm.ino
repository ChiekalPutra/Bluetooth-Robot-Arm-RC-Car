/*
 * =============================================
 *   ESP32 – Robot Arm 4 DOF via Bluetooth (Dabble)
 *   Kontrol: Dabble GamePad (tanpa motor)
 *
 *   Mapping tombol:
 *     DPad Kiri    → Base  kiri
 *     DPad Kanan   → Base  kanan
 *     Segitiga     → Bahu  naik
 *     X (Cross)    → Bahu  turun
 *     DPad Atas    → Siku  naik  (panjang)
 *     DPad Bawah→ Siku  turun (pendek)
 *     Kotak        → Capit buka
 *     Bulat        → Capit tutup
 *
 *   Smooth movement: target-based interpolasi,
 *   servo bergerak bertahap menuju posisi target.
 * =============================================
 */

#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>
#include <ESP32Servo.h>

// =============================================
//   PIN SERVO (GPIO yang aman di ESP32)
// =============================================
#define SERVO_BASE_PIN      25   // Base  – putar kiri/kanan
#define SERVO_SHOULDER_PIN  26   // Bahu  – naik/turun
#define SERVO_ELBOW_PIN     27   // Siku  – panjang/pendek
#define SERVO_GRIPPER_PIN   14   // Capit – buka/tutup

// =============================================
//   PARAMETER SERVO
// =============================================
#define SERVO_MIN        0
#define SERVO_MAX        180
#define GRIPPER_OPEN     30
#define GRIPPER_CLOSE    150

// Langkah target per iterasi (derajat) – kontrol kecepatan respons input
#define INPUT_STEP       3

// Langkah interpolasi per tick (derajat) – kontrol kehalusan gerakan
#define SMOOTH_STEP      1

// Interval update servo (ms)
#define SERVO_INTERVAL   15

// =============================================
//   OBJEK SERVO
// =============================================
Servo servoBase;
Servo servoShoulder;
Servo servoElbow;
Servo servoGripper;

// =============================================
//   STATE SERVO
//   current* = posisi fisik servo saat ini
//   target*  = posisi tujuan (diubah oleh input)
// =============================================
int currentBase     = 90,  targetBase     = 90;
int currentShoulder = 90,  targetShoulder = 90;
int currentElbow    = 90,  targetElbow    = 90;
int currentGripper  = 90,  targetGripper  = 90;

unsigned long lastServoUpdate = 0;

// =============================================
//   FUNGSI BANTU
// =============================================

// Batasi nilai dalam rentang [lo, hi]
inline int clamp(int val, int lo, int hi) {
  return val < lo ? lo : (val > hi ? hi : val);
}

// Gerakkan 'current' satu langkah menuju 'target'
inline int stepToward(int current, int target, int step) {
  if (current < target) return min(current + step, target);
  if (current > target) return max(current - step, target);
  return current;
}

// Tulis posisi terkini ke semua servo
void writeServos() {
  servoBase.write(currentBase);
  servoShoulder.write(currentShoulder);
  servoElbow.write(currentElbow);
  servoGripper.write(currentGripper);
}

// =============================================
//   BACA INPUT DAN UBAH TARGET
// =============================================
void handleInput() {
  // Base – DPad Kiri / Kanan
  if (GamePad.isLeftPressed())   targetBase = clamp(targetBase - INPUT_STEP, SERVO_MIN, SERVO_MAX);
  if (GamePad.isRightPressed()) targetBase = clamp(targetBase + INPUT_STEP, SERVO_MIN, SERVO_MAX);

  // Bahu – Segitiga naik / X turun
  if (GamePad.isTrianglePressed()) targetShoulder = clamp(targetShoulder + INPUT_STEP, SERVO_MIN, SERVO_MAX);
  if (GamePad.isCrossPressed())    targetShoulder = clamp(targetShoulder - INPUT_STEP, SERVO_MIN, SERVO_MAX);

  // Siku – DPad Atas naik / DPad Bawah turun
  if (GamePad.isUpPressed())  targetElbow = clamp(targetElbow + INPUT_STEP, SERVO_MIN, SERVO_MAX);
  if (GamePad.isDownPressed()) targetElbow = clamp(targetElbow - INPUT_STEP, SERVO_MIN, SERVO_MAX);

  // Capit – Kotak buka / Bulat tutup
  if (GamePad.isSquarePressed()) targetGripper = clamp(targetGripper - INPUT_STEP, GRIPPER_OPEN,  GRIPPER_CLOSE);
  if (GamePad.isCirclePressed()) targetGripper = clamp(targetGripper + INPUT_STEP, GRIPPER_OPEN,  GRIPPER_CLOSE);
}

// =============================================
//   INTERPOLASI HALUS MENUJU TARGET
// =============================================
void smoothUpdate() {
  currentBase     = stepToward(currentBase,     targetBase,     SMOOTH_STEP);
  currentShoulder = stepToward(currentShoulder, targetShoulder, SMOOTH_STEP);
  currentElbow    = stepToward(currentElbow,    targetElbow,    SMOOTH_STEP);
  currentGripper  = stepToward(currentGripper,  targetGripper,  SMOOTH_STEP);
  writeServos();
}

// =============================================
//   INISIALISASI SERVO
// =============================================
void setupServos() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servoBase.setPeriodHertz(50);
  servoShoulder.setPeriodHertz(50);
  servoElbow.setPeriodHertz(50);
  servoGripper.setPeriodHertz(50);

  servoBase.attach(SERVO_BASE_PIN,      500, 2400);
  servoShoulder.attach(SERVO_SHOULDER_PIN, 500, 2400);
  servoElbow.attach(SERVO_ELBOW_PIN,    500, 2400);
  servoGripper.attach(SERVO_GRIPPER_PIN, 500, 2400);

  writeServos(); // posisi awal
}

// =============================================
//   SETUP
// =============================================
void setup() {
  setupServos();
  Dabble.begin("RobotArm_ESP32");
}

// =============================================
//   LOOP
// =============================================
void loop() {
  Dabble.processInput();

  unsigned long now = millis();
  if (now - lastServoUpdate >= SERVO_INTERVAL) {
    lastServoUpdate = now;
    handleInput();   // baca tombol → ubah target
    smoothUpdate();  // geser current → target secara bertahap
  }
}
