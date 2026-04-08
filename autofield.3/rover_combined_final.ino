/*
  rover_combined_final.ino
  
  HOW IT WORKS
  ============
  Python (path_following.py) sends one line per control tick:
      "L1:<v>,L2:<v>,R1:<v>,R2:<v>,ARM:<v>\n"   (v = 0–255)

  The Arduino treats each value as a SPEED SETPOINT (0 = stop, 255 = max RPM).
  It scales 0–255 → 0–MAX_TARGET_RPM to get a targetRPM for each wheel, then
  the per-wheel P-controller adjusts the actual PWM every loop to make the
  measured RPM track the target. This corrects motor imbalance so the rover
  actually drives straight even when both sides are commanded the same value.

  ARMING HANDSHAKE
  ================
  1. Arduino sends  "READY"   on boot.
  2. Python sends   "START"   → Arduino replies "ARMED" and enables motors.
  3. Python sends   "STOP"    → Arduino disarms and cuts all motors.
  4. Watchdog: if no command arrives within TIMEOUT_MS, motors stop automatically.

  PIN ASSIGNMENTS (Arduino Uno)
  ==============================
  PWM output pins (must be hardware-PWM capable: 3,5,6,9,10,11 on Uno):
      LF = 3    LB = 5    RF = 6    RB = 11

  Direction pins (any digital GPIO):
      LF = 2    LB = 4    RF = 7    RB = 8

  Servo signal pin : 10
  Servo ref pin    : 12  (held LOW — only needed by some drivers, remove if not used)

  Hall-effect analog inputs:
      BL = A5   FL = A0   FR = A1   BR = A4


  TUNING CHECKLIST
  =================
  [ ] NUM_MAGNETS    — count of magnets glued to each wheel hub
  [ ] MAX_TARGET_RPM — free-running RPM at PWM=255 (measure with Serial Monitor)
  [ ] PWM_MIN        — lowest PWM that reliably overcomes stiction
  [ ] Kp_*           — start equal at ~1.5, raise if slow, lower if oscillating
  [ ] HALL_LOW/HIGH  — ADC window where your sensor reads "magnet present"
  [ ] SERVO_OFF/ON   — servo angles for arm up / arm down
  [ ] TIMEOUT_MS     — watchdog period; must be longer than your Python loop DT
*/

#include <Servo.h>


// ===========================================================================
// PIN DEFINITIONS
// ===========================================================================

// Motor PWM — hardware-PWM pins only on Uno (3, 5, 6, 9, 10, 11)
const int LF_PWM = 5;
const int LB_PWM = 3;
const int RF_PWM = 9;
const int RB_PWM = 6;

// Motor direction — any digital GPIO
const int LF_DIR = 4;
const int LB_DIR = 2;
const int RF_DIR = 8;
const int RB_DIR = 7;

// Servo
const int SERVO_SIG_PIN = 10;
const int SERVO_REF_PIN = 12;   // held LOW; remove wire if your driver doesn't need it

// Hall-effect analog inputs
const int HALL_BL = A5;
const int HALL_FL = A0;
const int HALL_FR = A1;
const int HALL_BR = A4;


// ===========================================================================
// SERVO CONFIG
// ===========================================================================

Servo paintArm;
const int SERVO_OFF_ANGLE = 70;    // arm up   — tune to your linkage geometry
const int SERVO_ON_ANGLE  = 40;   // arm down — tune to your linkage geometry
bool prevArmState = false; 

// ===========================================================================
// SAFETY / ARMING
// ===========================================================================

const unsigned long TIMEOUT_MS = 500;   // watchdog: ms between Python commands
const unsigned long SERVO_MOVE_MS = 400; // ms to hold servo signal while it travels, may need to tune
unsigned long lastCommandTime  = 0;
bool motorsStopped             = true;
bool armed                     = false;


// ===========================================================================
// HALL-EFFECT / P-CONTROLLER TUNING
// ===========================================================================

// Number of magnets attached to each wheel hub.
// Set this to match your actual hardware.
const int NUM_MAGNETS = 4;

// Free-running RPM at PWM=255 under no load.
// Run the rover on a stand, command full speed, read Serial Monitor, set here.
const float MAX_TARGET_RPM = 200.0f;

// Minimum PWM to overcome stiction before the P-term kicks in.
const int PWM_MIN = 30;

// ADC window that means "magnet is present in front of sensor".
// Your sensors read ~220–250 when a magnet is close; adjust if different.
const int HALL_LOW  = 200;
const int HALL_HIGH = 290;

// Per-wheel proportional gains.
// Start all equal; adjust individually if one wheel consistently leads/lags.
float Kp_BL = 1.5f;
float Kp_FL = 1.5f;
float Kp_FR = 1.5f;
float Kp_BR = 1.5f;


// ===========================================================================
// WHEEL STATE STRUCT
// Replaces the four separate blocks of copy-pasted variables in Prop_sensor_code
// ===========================================================================

struct WheelState {
  int           hallPin;
  int           pwmPin;
  float         Kp;              // per-wheel proportional gain

  // Hall-effect state
  bool          magnetPresent;
  bool          firstLoop;
  unsigned long passCount;
  unsigned long lastPassTime;
  float         rpm;

  // Controller state
  int           pwm;             // actual PWM currently written to the pin
  float         targetRPM;       // commanded by Python via serial
};

// Wheel order: [0]=BL  [1]=FL  [2]=FR  [3]=BR
// Matches the L2/L1/R1/R2 mapping in parseAndApply() below.
WheelState wheels[4] = {
  { HALL_BL, LB_PWM, 0.0f /*Kp set in setup*/, false, true, 0, 0, 0.0f, 0, 0.0f },
  { HALL_FL, LF_PWM, 0.0f, false, true, 0, 0, 0.0f, 0, 0.0f },
  { HALL_FR, RF_PWM, 0.0f, false, true, 0, 0, 0.0f, 0, 0.0f },
  { HALL_BR, RB_PWM, 0.0f, false, true, 0, 0, 0.0f, 0, 0.0f },
};


// ===========================================================================
// SETUP
// ===========================================================================

void setup() {
  Serial.begin(9600);

  // Assign per-wheel Kp from the tuning constants above
  wheels[0].Kp = Kp_BL;
  wheels[1].Kp = Kp_FL;
  wheels[2].Kp = Kp_FR;
  wheels[3].Kp = Kp_BR;

  // Drive PWM pins LOW before setting pinMode to avoid a boot glitch
  digitalWrite(LF_PWM, LOW);
  digitalWrite(LB_PWM, LOW);
  digitalWrite(RF_PWM, LOW);
  digitalWrite(RB_PWM, LOW);

  // Motor PWM pins
  pinMode(LF_PWM, OUTPUT);  analogWrite(LF_PWM, 0);
  pinMode(LB_PWM, OUTPUT);  analogWrite(LB_PWM, 0);
  pinMode(RF_PWM, OUTPUT);  analogWrite(RF_PWM, 0);
  pinMode(RB_PWM, OUTPUT);  analogWrite(RB_PWM, 0);

  // Motor direction pins — set LOW first while PWM is confirmed 0, then forward
  pinMode(LF_DIR, OUTPUT);  digitalWrite(LF_DIR, LOW);
  pinMode(LB_DIR, OUTPUT);  digitalWrite(LB_DIR, LOW);
  pinMode(RF_DIR, OUTPUT);  digitalWrite(RF_DIR, LOW);
  pinMode(RB_DIR, OUTPUT);  digitalWrite(RB_DIR, LOW);

  // Now safe to assert forward direction
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


// ===========================================================================
// MAIN LOOP
// ===========================================================================

void loop() {
  // 1. Read hall sensors, update RPM, run per-wheel P-controller
  updateAllWheels();

  // 2. Parse any incoming serial command from Python
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    // --- Arming handshake ---
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

    // --- Disarm ---
    if (line == "STOP") {
      armed = false;
      stopAll();
      Serial.println("DISARMED");
      return;
    }

    // --- Motor + servo command ---
    parseAndApply(line);
  }

  // 3. Watchdog: stop motors if Python goes silent
  if (armed && !motorsStopped &&
      (millis() - lastCommandTime > TIMEOUT_MS)) {
    stopAll();
    Serial.println("WATCHDOG:TIMEOUT");
  }
}


// ===========================================================================
// HALL-EFFECT READING + PER-WHEEL P-CONTROLLER
//
// Called every loop iteration.
// Reads each sensor, detects magnet rising-edge, computes RPM, then adjusts
// PWM to drive measured RPM toward targetRPM.
//
// Key difference from Prop_sensor_code:
//   - Uses the struct array instead of four copy-pasted variable blocks.
//   - RPM formula uses NUM_MAGNETS so you only change one constant above.
//   - P-controller integrates (+=) rather than computing from scratch each
//     tick, so it doesn't slam the motors when error is large.
// ===========================================================================

void updateAllWheels() {
  if (!armed || motorsStopped) return;   // don't touch motors while disarmed

  for (int i = 0; i < 4; i++) {
    WheelState &w = wheels[i];

    int  analogVal      = analogRead(w.hallPin);
    bool magnetDetected = (analogVal >= HALL_LOW && analogVal <= HALL_HIGH);

    // Initialise on the very first call
    if (w.firstLoop) {
      w.magnetPresent = magnetDetected;
      w.lastPassTime  = millis();
      w.firstLoop     = false;
      continue;
    }

    // Rising edge — magnet just arrived in front of sensor
    if (magnetDetected && !w.magnetPresent) {
      unsigned long now           = millis();
      unsigned long timeBetween   = now - w.lastPassTime;
      w.lastPassTime              = now;

      if (timeBetween > 0) {
        // 60000 ms/min ÷ NUM_MAGNETS pulses/rev ÷ timeBetween ms/pulse
        w.rpm = (60000.0f / float(NUM_MAGNETS)) / float(timeBetween);
      }

      w.passCount++;
      w.magnetPresent = true;
    }

    // Falling edge
    if (!magnetDetected && w.magnetPresent) {
      w.magnetPresent = false;
    }

    // Staleness: if no pulse for > 1 s, the wheel has stopped
    if (millis() - w.lastPassTime > 1000) {
      w.rpm = 0.0f;
    }

    // --- Proportional controller ---
    if (w.targetRPM < 1.0f) {
      // Python commanded zero — hard stop this wheel
      w.pwm = 0;
    } else {
      float error  = w.targetRPM - w.rpm;
      w.pwm       += int(w.Kp * error);
      // Clamp: PWM_MIN ensures the motor doesn't stall at small errors;
      // 255 is the hardware ceiling.
      w.pwm = constrain(w.pwm, PWM_MIN, 255);
    }

    analogWrite(w.pwmPin, w.pwm);
  }
}

// ===========================================================================
// SET SERVO POSITION
// Moves servo to angle, waits for travel, then detaches to go passive.
// ===========================================================================

void setServoPosition(int angle) {
  paintArm.attach(SERVO_SIG_PIN);
  paintArm.write(angle);
  delay(SERVO_MOVE_MS);
  paintArm.detach();
}

// ===========================================================================
// COMMAND PARSER
//
// Expects: "L1:<v>,L2:<v>,R1:<v>,R2:<v>,ARM:<v>"   v = 0–255
//
// L1 = front-left setpoint,  L2 = back-left setpoint
// R1 = front-right setpoint, R2 = back-right setpoint
//
// Each setpoint is scaled to a targetRPM for its wheel:
//     targetRPM = (setpoint / 255) * MAX_TARGET_RPM
// ===========================================================================

// Scale a Python PWM setpoint (0–255) to a target RPM
float pwmToTargetRPM(int setpoint) {
  return (float(setpoint) / 255.0f) * MAX_TARGET_RPM;
}

void parseAndApply(String cmd) {
  int l1  = extractValue(cmd, "L1:");
  int l2  = extractValue(cmd, "L2:");
  int r1  = extractValue(cmd, "R1:");
  int r2  = extractValue(cmd, "R2:");
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

  // Map setpoints to per-wheel target RPM
  // wheels[0]=BL  wheels[1]=FL  wheels[2]=FR  wheels[3]=BR
  wheels[0].targetRPM = pwmToTargetRPM(l2);   // Back-Left   ← L2
  wheels[1].targetRPM = pwmToTargetRPM(l1);   // Front-Left  ← L1
  wheels[2].targetRPM = pwmToTargetRPM(r1);   // Front-Right ← R1
  wheels[3].targetRPM = pwmToTargetRPM(r2);   // Back-Right  ← R2

  bool newArmState = (arm == 1);
  if (newArmState != prevArmState) {   // only act on a STATE CHANGE
   if (newArmState) {
     setServoPosition(SERVO_ON_ANGLE);  // lower arm, then go passive
   } else {
      setServoPosition(SERVO_OFF_ANGLE); // raise arm, then go passive
    }
   prevArmState = newArmState;
  }

  lastCommandTime = millis();
  motorsStopped   = false;

  // Echo for debugging — also reports current measured RPM per wheel
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


// ===========================================================================
// VALUE EXTRACTOR
// Pulls the integer after a key string like "L1:" from a comma-separated line.
// Returns -1 if the key is not found.
// ===========================================================================

int extractValue(String cmd, String key) {
  int keyIndex = cmd.indexOf(key);
  if (keyIndex < 0) return -1;

  int    start  = keyIndex + key.length();
  int    end    = cmd.indexOf(',', start);
  String valStr = (end < 0) ? cmd.substring(start)
                             : cmd.substring(start, end);
  valStr.trim();
  return valStr.toInt();
}


// ===========================================================================
// STOP ALL MOTORS AND SERVO SAFELY
// ===========================================================================

void stopAll() {
  for (int i = 0; i < 4; i++) {
    wheels[i].targetRPM = 0.0f;
    wheels[i].pwm       = 0;
    analogWrite(wheels[i].pwmPin, 0);
  }
  paintArm.write(SERVO_OFF_ANGLE);
  motorsStopped = true;
}
