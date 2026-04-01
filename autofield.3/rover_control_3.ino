/*
  rover_control.ino

  Receives serial commands from path_following.py and drives:
    - 4 x Cytron 30A (MDD30A) motor drivers (PWM + DIR per motor)
    - 1 x servo (paint arm) via digital HIGH/LOW signal

  Serial format (from Python):
    "L1:<v>,L2:<v>,R1:<v>,R2:<v>,ARM:<v>\n"
    where <v> is 0–255 for motors, 0 or 1 for ARM.

  Cytron 30A wiring per motor:
    PWM pin → speed (0–255 via analogWrite)
    DIR pin → direction (HIGH = forward, LOW = reverse)

  Pin assignments:
    LF (Left  Front) : PWM=2,  DIR=3
    LB (Left  Back)  : PWM=4,  DIR=5
    RF (Right Front) : PWM=6,  DIR=10
    RB (Right Back)  : PWM=8,  DIR=9
    Servo signal     : pin 11
    Servo reference  : pin 12 (held LOW — use if driver needs a ref pin)

  All motors run forward only for test day (DIR pins held HIGH).
  Watchdog: if no valid command received within TIMEOUT_MS,
  all motors stop automatically.
*/

#include <Servo.h>

// ---------------------------------------------------------------------------
// Pin definitions
// ---------------------------------------------------------------------------
// Each motor: {PWM_PIN, DIR_PIN}
const int LF_PWM = 2,  LF_DIR = 3;
const int LB_PWM = 4,  LB_DIR = 5;
const int RF_PWM = 6,  RF_DIR = 10;
const int RB_PWM = 8,  RB_DIR = 9;

const int SERVO_SIG_PIN = 11;
const int SERVO_REF_PIN = 12;   // held LOW — remove if not needed

// ---------------------------------------------------------------------------
// Servo config
// ---------------------------------------------------------------------------
Servo paintArm;
const int SERVO_OFF_ANGLE = 0;    // degrees — arm up   (adjust to your linkage)
const int SERVO_ON_ANGLE  = 90;   // degrees — arm down (adjust to your linkage)

// ---------------------------------------------------------------------------
// Safety watchdog
// ---------------------------------------------------------------------------
const unsigned long TIMEOUT_MS = 500;  // stop if no command for 500 ms
unsigned long lastCommandTime  = 0;
bool motorsStopped             = true;

// ---------------------------------------------------------------------------
// Arming state — motors locked until Python sends "START"
// ---------------------------------------------------------------------------
bool armed = false;

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);

  // Drive all PWM pins LOW first — before pinMode or DIR pins are set.
  // This prevents Cytron drivers from seeing a floating signal on boot.
  digitalWrite(LF_PWM, LOW);
  digitalWrite(LB_PWM, LOW);
  digitalWrite(RF_PWM, LOW);
  digitalWrite(RB_PWM, LOW);

  // Motor PWM pins
  pinMode(LF_PWM, OUTPUT);  digitalWrite(LF_PWM, LOW);
  pinMode(LB_PWM, OUTPUT);  digitalWrite(LB_PWM, LOW);
  pinMode(RF_PWM, OUTPUT);  digitalWrite(RF_PWM, LOW);
  pinMode(RB_PWM, OUTPUT);  digitalWrite(RB_PWM, LOW);

  // Motor DIR pins — set LOW first, then HIGH after PWM is confirmed 0
  pinMode(LF_DIR, OUTPUT);  digitalWrite(LF_DIR, LOW);
  pinMode(LB_DIR, OUTPUT);  digitalWrite(LB_DIR, LOW);
  pinMode(RF_DIR, OUTPUT);  digitalWrite(RF_DIR, LOW);
  pinMode(RB_DIR, OUTPUT);  digitalWrite(RB_DIR, LOW);

  // Explicitly zero all PWM channels before enabling direction
  analogWrite(LF_PWM, 0);
  analogWrite(LB_PWM, 0);
  analogWrite(RF_PWM, 0);
  analogWrite(RB_PWM, 0);

  // Now safe to set direction pins HIGH (forward)
  digitalWrite(LF_DIR, HIGH);
  digitalWrite(LB_DIR, HIGH);
  digitalWrite(RF_DIR, HIGH);
  digitalWrite(RB_DIR, HIGH);

  // Servo
  paintArm.attach(SERVO_SIG_PIN);
  paintArm.write(SERVO_OFF_ANGLE);

  pinMode(SERVO_REF_PIN, OUTPUT);
  digitalWrite(SERVO_REF_PIN, LOW);

  // Confirm stopped state
  stopAll();

  // Wait for Python to send START before accepting any motor commands
  Serial.println("READY");
  Serial.println("INFO:Waiting for START command");
}

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------
void loop() {
  // Read incoming serial command
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    // Arming handshake — must receive START before motors respond
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

    // Disarm command — stops motors and returns to waiting state
    if (line == "STOP") {
      armed = false;
      stopAll();
      Serial.println("DISARMED");
      return;
    }

    parseAndApply(line);
  }

  // Watchdog — stop motors if Python side goes quiet
  if (armed && !motorsStopped &&
      (millis() - lastCommandTime > TIMEOUT_MS)) {
    stopAll();
    Serial.println("WATCHDOG:TIMEOUT");
  }
}

// ---------------------------------------------------------------------------
// Command parser
// Expects: "L1:<v>,L2:<v>,R1:<v>,R2:<v>,ARM:<v>"
// ---------------------------------------------------------------------------
void parseAndApply(String cmd) {
  int l1 = -1, l2 = -1, r1 = -1, r2 = -1, arm = -1;

  // Parse each field
  l1  = extractValue(cmd, "L1:");
  l2  = extractValue(cmd, "L2:");
  r1  = extractValue(cmd, "R1:");
  r2  = extractValue(cmd, "R2:");
  arm = extractValue(cmd, "ARM:");

  // Validate — reject if any field is missing
  if (l1 < 0 || l2 < 0 || r1 < 0 || r2 < 0 || arm < 0) {
    Serial.print("ERR:BAD_CMD:");
    Serial.println(cmd);
    return;
  }

  // Clamp PWM to valid range
  l1  = constrain(l1,  0, 255);
  l2  = constrain(l2,  0, 255);
  r1  = constrain(r1,  0, 255);
  r2  = constrain(r2,  0, 255);
  arm = constrain(arm, 0, 1);

  // Apply to motors
  analogWrite(LF_PWM, l1);
  analogWrite(LB_PWM, l2);
  analogWrite(RF_PWM, r1);
  analogWrite(RB_PWM, r2);

  // Apply to servo
  paintArm.write(arm == 1 ? SERVO_ON_ANGLE : SERVO_OFF_ANGLE);

  lastCommandTime = millis();
  motorsStopped   = false;

  // Echo back for debugging
  Serial.print("OK:L1="); Serial.print(l1);
  Serial.print(",L2=");    Serial.print(l2);
  Serial.print(",R1=");    Serial.print(r1);
  Serial.print(",R2=");    Serial.print(r2);
  Serial.print(",ARM=");   Serial.println(arm);
}

// ---------------------------------------------------------------------------
// Extract integer value after a key like "L1:"
// Returns -1 if key not found
// ---------------------------------------------------------------------------
int extractValue(String cmd, String key) {
  int keyIndex = cmd.indexOf(key);
  if (keyIndex < 0) return -1;

  int start = keyIndex + key.length();
  int end   = cmd.indexOf(',', start);   // find next comma
  String valStr = (end < 0)
      ? cmd.substring(start)             // last field — no trailing comma
      : cmd.substring(start, end);

  valStr.trim();
  return valStr.toInt();
}

// ---------------------------------------------------------------------------
// Stop everything safely
// ---------------------------------------------------------------------------
void stopAll() {
  analogWrite(LF_PWM, 0);
  analogWrite(LB_PWM, 0);
  analogWrite(RF_PWM, 0);
  analogWrite(RB_PWM, 0);
  paintArm.write(SERVO_OFF_ANGLE);
  motorsStopped = true;
}
