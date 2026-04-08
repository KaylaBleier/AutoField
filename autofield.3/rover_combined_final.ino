#include <Servo.h>

// ===========================================================================
// PIN DEFINITIONS
// ===========================================================================

const int LF_PWM = 5;
const int LB_PWM = 3;
const int RF_PWM = 9;
const int RB_PWM = 6;

const int LF_DIR = 4;
const int LB_DIR = 2;
const int RF_DIR = 8;
const int RB_DIR = 7;

const int SERVO_SIG_PIN = 12;
//const int SERVO_REF_PIN = 12;

const int HALL_BL = A5;
const int HALL_FL = A0;
const int HALL_FR = A1;
const int HALL_BR = A4;

// ===========================================================================
// SERVO
// ===========================================================================
Servo paintArm;
const int SERVO_OFF_ANGLE = 0; //no paint
const int SERVO_ON_ANGLE  = 29; //painting
bool prevArmState = false;

// ===========================================================================
// SAFETY
// ===========================================================================
const unsigned long TIMEOUT_MS = 500;
const unsigned long SERVO_MOVE_MS = 400;
unsigned long lastCommandTime = 0;
bool motorsStopped = true;
bool armed = false;

// ================= NEW =================
unsigned long lastEncSend = 0;
const unsigned long ENC_INTERVAL_MS = 100;
// =======================================

// ===========================================================================
// CONTROL TUNING
// ===========================================================================
const int NUM_MAGNETS = 4;
const float MAX_TARGET_RPM = 200.0f;
const int PWM_MIN = 30;

const int HALL_LOW  = 200;
const int HALL_HIGH = 290;

float Kp_BL = 1.5f;
float Kp_FL = 1.5f;
float Kp_FR = 1.5f;
float Kp_BR = 1.5f;

// ===========================================================================
// WHEEL STRUCT
// ===========================================================================
struct WheelState {
  int hallPin;
  int pwmPin;
  float Kp;

  bool magnetPresent;
  bool firstLoop;
  unsigned long passCount;
  unsigned long lastPassTime;
  float rpm;

  int pwm;
  float targetRPM;
};

WheelState wheels[4] = {
  { HALL_BL, LB_PWM, 0, false, true, 0, 0, 0, 0, 0 },
  { HALL_FL, LF_PWM, 0, false, true, 0, 0, 0, 0, 0 },
  { HALL_FR, RF_PWM, 0, false, true, 0, 0, 0, 0, 0 },
  { HALL_BR, RB_PWM, 0, false, true, 0, 0, 0, 0, 0 },
};

// ===========================================================================
// SETUP
// ===========================================================================
void setup() {
  Serial.begin(9600);

  wheels[0].Kp = Kp_BL;
  wheels[1].Kp = Kp_FL;
  wheels[2].Kp = Kp_FR;
  wheels[3].Kp = Kp_BR;

  pinMode(LF_PWM, OUTPUT);
  pinMode(LB_PWM, OUTPUT);
  pinMode(RF_PWM, OUTPUT);
  pinMode(RB_PWM, OUTPUT);

  analogWrite(LF_PWM, 0);
  analogWrite(LB_PWM, 0);
  analogWrite(RF_PWM, 0);
  analogWrite(RB_PWM, 0);

  pinMode(LF_DIR, OUTPUT);
  pinMode(LB_DIR, OUTPUT);
  pinMode(RF_DIR, OUTPUT);
  pinMode(RB_DIR, OUTPUT);

  digitalWrite(LF_DIR, HIGH);
  digitalWrite(LB_DIR, HIGH);
  digitalWrite(RF_DIR, HIGH);
  digitalWrite(RB_DIR, HIGH);

  paintArm.attach(SERVO_SIG_PIN);
  paintArm.write(SERVO_OFF_ANGLE);

  pinMode(SERVO_REF_PIN, OUTPUT);
  digitalWrite(SERVO_REF_PIN, LOW);

  stopAll();

  Serial.println("READY");
}

// ===========================================================================
// LOOP
// ===========================================================================
void loop() {

  updateAllWheels();

  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (!armed) {
      if (line == "START") {
        armed = true;
        lastCommandTime = millis();
        Serial.println("ARMED");
      }
      return;
    }

    if (line == "STOP") {
      armed = false;
      stopAll();
      Serial.println("DISARMED");
      return;
    }

    parseAndApply(line);
  }

  if (armed && !motorsStopped &&
      (millis() - lastCommandTime > TIMEOUT_MS)) {
    stopAll();
    Serial.println("WATCHDOG");
  }

  // ===== NEW ENCODER OUTPUT =====
  if (millis() - lastEncSend > ENC_INTERVAL_MS) {
    sendEncoderData();
    lastEncSend = millis();
  }
}

// ===========================================================================
// ENCODER OUTPUT
// ===========================================================================
void sendEncoderData() {
  Serial.print("ENC:");

  Serial.print("BL:");
  Serial.print(wheels[0].passCount);
  Serial.print(",RPM:");
  Serial.print(wheels[0].rpm, 1);

  Serial.print(",FL:");
  Serial.print(wheels[1].passCount);
  Serial.print(",RPM:");
  Serial.print(wheels[1].rpm, 1);

  Serial.print(",FR:");
  Serial.print(wheels[2].passCount);
  Serial.print(",RPM:");
  Serial.print(wheels[2].rpm, 1);

  Serial.print(",BR:");
  Serial.print(wheels[3].passCount);
  Serial.print(",RPM:");
  Serial.print(wheels[3].rpm, 1);

  Serial.println();
}

// ===========================================================================
// CONTROL LOOP
// ===========================================================================
void updateAllWheels() {
  if (!armed || motorsStopped) return;

  for (int i = 0; i < 4; i++) {
    WheelState &w = wheels[i];

    int val = analogRead(w.hallPin);
    bool detected = (val >= HALL_LOW && val <= HALL_HIGH);

    if (w.firstLoop) {
      w.magnetPresent = detected;
      w.lastPassTime = millis();
      w.firstLoop = false;
      continue;
    }

    if (detected && !w.magnetPresent) {
      unsigned long now = millis();
      unsigned long dt = now - w.lastPassTime;
      w.lastPassTime = now;

      if (dt > 0) {
        w.rpm = (60000.0f / NUM_MAGNETS) / dt;
      }

      w.passCount++;
      w.magnetPresent = true;
    }

    if (!detected && w.magnetPresent) {
      w.magnetPresent = false;
    }

    if (millis() - w.lastPassTime > 1000) {
      w.rpm = 0;
    }

    if (w.targetRPM < 1) {
      w.pwm = 0;
    } else {
      float error = w.targetRPM - w.rpm;
      w.pwm += int(w.Kp * error);
      w.pwm = constrain(w.pwm, PWM_MIN, 255);
    }

    analogWrite(w.pwmPin, w.pwm);
  }
}

// ===========================================================================
// COMMAND PARSER
// ===========================================================================
float pwmToTargetRPM(int v) {
  return (float(v) / 255.0f) * MAX_TARGET_RPM;
}

void parseAndApply(String cmd) {
  int l1 = extractValue(cmd, "L1:");
  int l2 = extractValue(cmd, "L2:");
  int r1 = extractValue(cmd, "R1:");
  int r2 = extractValue(cmd, "R2:");
  int arm = extractValue(cmd, "ARM:");

  if (l1 < 0 || l2 < 0 || r1 < 0 || r2 < 0) return;

  wheels[0].targetRPM = pwmToTargetRPM(l2);
  wheels[1].targetRPM = pwmToTargetRPM(l1);
  wheels[2].targetRPM = pwmToTargetRPM(r1);
  wheels[3].targetRPM = pwmToTargetRPM(r2);

  lastCommandTime = millis();
  motorsStopped = false;
}

// ===========================================================================
// HELPERS
// ===========================================================================
int extractValue(String cmd, String key) {
  int i = cmd.indexOf(key);
  if (i < 0) return -1;
  int start = i + key.length();
  int end = cmd.indexOf(',', start);
  String val = (end < 0) ? cmd.substring(start) : cmd.substring(start, end);
  return val.toInt();
}

void stopAll() {
  for (int i = 0; i < 4; i++) {
    wheels[i].targetRPM = 0;
    wheels[i].pwm = 0;
    analogWrite(wheels[i].pwmPin, 0);
  }
  motorsStopped = true;
}
