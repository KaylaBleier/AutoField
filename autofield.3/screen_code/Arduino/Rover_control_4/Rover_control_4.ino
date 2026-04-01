/*
  rover_combined.ino

  Combines rover_control_3.ino  (serial command parsing, Cytron drivers, servo)
  with HallEffectSensor_code.ino (hall-effect RPM reading, per-wheel P-loop).

  HOW IT WORKS
  ============
  Python (path_following.py) sends:
      "L1:<v>,L2:<v>,R1:<v>,R2:<v>,ARM:<v>\n"   (v = 0–255)

  The Arduino treats each PWM value as a SPEED SETPOINT, not a direct drive.
  It scales 0–255  →  0–MAX_TARGET_RPM to get targetRPM for each wheel,
  then the per-wheel P-controller adjusts actual PWM up/down every loop
  to make the measured RPM track that target. This corrects for motor
  imbalance so the rover actually drives straight.

  ARMING HANDSHAKE (unchanged from rover_control_3)
  ==================================================
  1. Arduino sends  "READY"   on boot.
  2. Python sends   "START"   → Arduino replies "ARMED" and enables motors.
  3. Python sends   "STOP"    → Arduino disarms and cuts motors.
  4. Watchdog: if no command arrives within TIMEOUT_MS, motors stop.

  PIN ASSIGNMENTS  (Arduino Uno)
  ==============================
  Motor PWM  (all true PWM pins):
      LF = 3   LB = 5   RF = 6   RB = 11

  Motor DIR  (any GPIO):
      LF = 2   LB = 4   RF = 7   RB = 8

  Servo signal: 10
  Servo ref   : 12  (held LOW — remove if not needed)

  Hall-effect analog inputs:
      BL = A5   FL = A0   FR = A1   BR = A4

  WHY THE PINS CHANGED FROM THE ORIGINALS
  =========================================
  rover_control_3 had PWM on pins 2, 4, 8 — none of those are PWM-capable
  on the Uno. They are corrected here to 3, 5, 6, 11.

  The old DIR pins (3, 5, 9, 10) conflicted with the new PWM pins and the
  Servo library (which needs pin 9 or 10). DIR pins are now 2, 4, 7, 8 —
  all plain GPIO, no conflicts.

  The servo moved from pin 11 to pin 10 to free pin 11 for RB motor PWM.

  TUNING
  ======
  Kp            — raise if the rover is slow to correct speed differences;
                  lower if wheels oscillate or hunt.
  MAX_TARGET_RPM — set to the free-running RPM of your motors at full PWM.
  PWM_MIN        — minimum PWM that keeps motors turning (deadband offset).
  TIMEOUT_MS     — watchdog timeout in ms.
*/

#include <Servo.h>

// ---------------------------------------------------------------------------
// Pin definitions
// ---------------------------------------------------------------------------

// Motor PWM (must be hardware-PWM pins on Uno: 3, 5, 6, 9, 10, 11)
const int LF_PWM = 3;
const int LB_PWM = 5;
const int RF_PWM = 6;
const int RB_PWM = 11;

// Motor direction (any digital GPIO)
const int LF_DIR = 2;
const int LB_DIR = 4;
const int RF_DIR = 7;
const int RB_DIR = 8;

// Servo
const int SERVO_SIG_PIN = 10;
const int SERVO_REF_PIN = 12;   // held LOW — remove wire if driver doesn't need it

// Hall-effect analog inputs
const int HALL_BL = A5;
const int HALL_FL = A0;
const int HALL_FR = A1;
const int HALL_BR = A4;

// ---------------------------------------------------------------------------
// Servo config
// ---------------------------------------------------------------------------
Servo paintArm;
const int SERVO_OFF_ANGLE = 0;    // arm up   — adjust to your linkage
const int SERVO_ON_ANGLE  = 90;   // arm down — adjust to your linkage

// ---------------------------------------------------------------------------
// Safety watchdog
// ---------------------------------------------------------------------------
const unsigned long TIMEOUT_MS = 500;
unsigned long lastCommandTime  = 0;
bool motorsStopped             = true;

// ---------------------------------------------------------------------------
// Arming state
// ---------------------------------------------------------------------------
bool armed = false;

// ---------------------------------------------------------------------------
// Hall-effect state — one block per wheel
// ---------------------------------------------------------------------------
struct WheelState {
  int           hallPin;
  bool          magnetPresent;
  bool          firstLoop;
  unsigned long passCount;
  unsigned long lastPassTime;
  unsigned long timeBetweenPasses;
  float         rpm;
  int           pwmPin;
  int           pwm;          // current actual PWM being written
  float         targetRPM;    // commanded RPM from Python
};

WheelState wheels[4] = {
  // {hallPin, magnetPresent, firstLoop, passCount, lastPassTime, timeBetweenPasses, rpm, pwmPin, pwm, targetRPM}
  { HALL_BL, false, true, 0, 0, 0, 0.0f, LB_PWM, 0, 0.0f },   // [0] Back-Left
  { HALL_FL, false, true, 0, 0, 0, 0.0f, LF_PWM, 0, 0.0f },   // [1] Front-Left
  { HALL_FR, false, true, 0, 0, 0, 0.0f, RF_PWM, 0, 0.0f },   // [2] Front-Right
  { HALL_BR, false, true, 0, 0, 0, 0.0f, RB_PWM, 0, 0.0f },   // [3] Back-Right
};

// ---------------------------------------------------------------------------
// P-controller tuning
// ---------------------------------------------------------------------------

// Maximum RPM your motors reach at full throttle (PWM=255, no load).
// Measure this with the hall sensors and Serial Monitor, then set it here.
const float MAX_TARGET_RPM = 200.0f;

// Proportional gain. Start at 0.5 and increase until tracking is crisp
// without oscillation. Each wheel can be tuned separately if needed.
const float Kp = 0.8f;

// Minimum PWM to overcome stiction before the P-term kicks in.
// Set to the lowest PWM value that makes your motors turn reliably.
const int PWM_MIN = 30;

// Hall threshold — sensor returns ~220–250 when magnet is present.
// Adjust if your sensor's quiescent value is different.
const int HALL_LOW  = 210;
const int HALL_HIGH = 260;

// ---------------------------------------------------------------------------
// Helper: scale Python PWM setpoint (0–255) → target RPM
// ---------------------------------------------------------------------------
float pwmToTargetRPM(int pwmSetpoint) {
  return (float(pwmSetpoint) / 255.0f) * MAX_TARGET_RPM;
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);

  // Drive PWM pins LOW before setting pinMode to prevent glitch on boot
  digitalWrite(LF_PWM, LOW);
  digitalWrite(LB_PWM, LOW);
  digitalWrite(RF_PWM, LOW);
  digitalWrite(RB_PWM, LOW);

  // Motor PWM pins
  pinMode(LF_PWM, OUTPUT);  analogWrite(LF_PWM, 0);
  pinMode(LB_PWM, OUTPUT);  analogWrite(LB_PWM, 0);
  pinMode(RF_PWM, OUTPUT);  analogWrite(RF_PWM, 0);
  pinMode(RB_PWM, OUTPUT);  analogWrite(RB_PWM, 0);

  // Motor direction pins — LOW first, then HIGH (forward) after PWM is 0
  pinMode(LF_DIR, OUTPUT);  digitalWrite(LF_DIR, LOW);
  pinMode(LB_DIR, OUTPUT);  digitalWrite(LB_DIR, LOW);
  pinMode(RF_DIR, OUTPUT);  digitalWrite(RF_DIR, LOW);
  pinMode(RB_DIR, OUTPUT);  digitalWrite(RB_DIR, LOW);

  // All motors confirmed stopped — now safe to assert forward direction
  digitalWrite(LF_DIR, HIGH);
  digitalWrite(LB_DIR, HIGH);
  digitalWrite(RF_DIR, HIGH);
  digitalWrite(RB_DIR, HIGH);

  // Servo
  paintArm.attach(SERVO_SIG_PIN);
  paintArm.write(SERVO_OFF_ANGLE);

  pinMode(SERVO_REF_PIN, OUTPUT);
  digitalWrite(SERVO_REF_PIN, LOW);

  stopAll();

  Serial.println("READY");
  Serial.println("INFO:Waiting for START command");
}

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------
void loop() {
  // 1. Read hall sensors and update RPM + run P-controller for all wheels
  updateAllWheels();

  // 2. Parse any incoming serial command
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    // Arming handshake
    if (!armed) {
      if (line == "START") {
        armed = true;
        lastCommandTime = millis();
        Serial.println("ARMED");
      } else {
        Serial.println("ERR:NOT_ARMED — send START first");
      }
      return;
    }

    // Disarm
    if (line == "STOP") {
      armed = false;
      stopAll();
      Serial.println("DISARMED");
      return;
    }

    parseAndApply(line);
  }

  // 3. Watchdog
  if (armed && !motorsStopped &&
      (millis() - lastCommandTime > TIMEOUT_MS)) {
    stopAll();
    Serial.println("WATCHDOG:TIMEOUT");
  }
}

// ---------------------------------------------------------------------------
// Hall-effect reading + P-controller for all four wheels
// ---------------------------------------------------------------------------
void updateAllWheels() {
  if (!armed || motorsStopped) return;   // don't spin wheels while disarmed

  for (int i = 0; i < 4; i++) {
    WheelState &w = wheels[i];

    int analogVal = analogRead(w.hallPin);
    bool magnetDetected = (analogVal >= HALL_LOW && analogVal <= HALL_HIGH);

    // Initialise on the very first pass
    if (w.firstLoop) {
      w.magnetPresent = magnetDetected;
      w.lastPassTime  = millis();
      w.firstLoop     = false;
      continue;
    }

    // Rising edge — magnet just arrived
    if (magnetDetected && !w.magnetPresent) {
      unsigned long now        = millis();
      w.timeBetweenPasses      = now - w.lastPassTime;
      w.lastPassTime           = now;
      if (w.timeBetweenPasses > 0) {
        w.rpm = 60000.0f / float(w.timeBetweenPasses);
      }
      w.passCount++;
      w.magnetPresent = true;
    }

    // Falling edge
    if (!magnetDetected && w.magnetPresent) {
      w.magnetPresent = false;
    }

    // RPM staleness: if no pulse for >1 s, assume motor has stopped
    if (millis() - w.lastPassTime > 1000) {
      w.rpm = 0.0f;
    }

    // --- Proportional controller ---
    // Only run if we actually want the wheel to move
    if (w.targetRPM < 1.0f) {
      // Python commanded 0 — hard stop this wheel
      w.pwm = 0;
    } else {
      float error   = w.targetRPM - w.rpm;
      float adjust  = Kp * error;
      w.pwm += int(adjust);

      // Clamp: keep above PWM_MIN so the motor doesn't stall at small errors,
      // but only when actually commanded to move.
      w.pwm = constrain(w.pwm, PWM_MIN, 255);
    }

    analogWrite(w.pwmPin, w.pwm);
  }
}

// ---------------------------------------------------------------------------
// Command parser
// Expects: "L1:<v>,L2:<v>,R1:<v>,R2:<v>,ARM:<v>"
//
// L1/L2 are the left-side setpoints; R1/R2 the right-side setpoints.
// These are converted to targetRPM for each of the four wheels.
// ---------------------------------------------------------------------------
void parseAndApply(String cmd) {
  int l1 = extractValue(cmd, "L1:");
  int l2 = extractValue(cmd, "L2:");
  int r1 = extractValue(cmd, "R1:");
  int r2 = extractValue(cmd, "R2:");
  int arm = extractValue(cmd, "ARM:");

  if (l1 < 0 || l2 < 0 || r1 < 0 || r2 < 0 || arm < 0) {
    Serial.print("ERR:BAD_CMD:");
    Serial.println(cmd);
    return;
  }

  l1  = constrain(l1,  0, 255);
  l2  = constrain(l2,  0, 255);
  r1  = constrain(r1,  0, 255);
  r2  = constrain(r2,  0, 255);
  arm = constrain(arm, 0, 1);

  // Map Python PWM setpoints to target RPM for each wheel
  // wheel index: [0]=BL, [1]=FL, [2]=FR, [3]=BR
  wheels[0].targetRPM = pwmToTargetRPM(l2);   // Back-Left  ← L2
  wheels[1].targetRPM = pwmToTargetRPM(l1);   // Front-Left ← L1
  wheels[2].targetRPM = pwmToTargetRPM(r1);   // Front-Right← R1
  wheels[3].targetRPM = pwmToTargetRPM(r2);   // Back-Right ← R2

  // Servo
  paintArm.write(arm == 1 ? SERVO_ON_ANGLE : SERVO_OFF_ANGLE);

  lastCommandTime = millis();
  motorsStopped   = false;

  // Echo for debugging — also reports current measured RPM
  Serial.print("OK:L1="); Serial.print(l1);
  Serial.print(",L2=");   Serial.print(l2);
  Serial.print(",R1=");   Serial.print(r1);
  Serial.print(",R2=");   Serial.print(r2);
  Serial.print(",ARM=");  Serial.print(arm);
  Serial.print(" | RPM BL="); Serial.print(wheels[0].rpm, 0);
  Serial.print(" FL=");       Serial.print(wheels[1].rpm, 0);
  Serial.print(" FR=");       Serial.print(wheels[2].rpm, 0);
  Serial.print(" BR=");       Serial.println(wheels[3].rpm, 0);
}

// ---------------------------------------------------------------------------
// Extract integer value after a key like "L1:"
// Returns -1 if key not found
// ---------------------------------------------------------------------------
int extractValue(String cmd, String key) {
  int keyIndex = cmd.indexOf(key);
  if (keyIndex < 0) return -1;

  int start   = keyIndex + key.length();
  int end     = cmd.indexOf(',', start);
  String valStr = (end < 0) ? cmd.substring(start)
                             : cmd.substring(start, end);
  valStr.trim();
  return valStr.toInt();
}

// ---------------------------------------------------------------------------
// Stop everything safely
// ---------------------------------------------------------------------------
void stopAll() {
  for (int i = 0; i < 4; i++) {
    wheels[i].targetRPM = 0.0f;
    wheels[i].pwm       = 0;
    analogWrite(wheels[i].pwmPin, 0);
  }
  paintArm.write(SERVO_OFF_ANGLE);
  motorsStopped = true;
}
