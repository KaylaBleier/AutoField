#include <Servo.h>

// ========================
// PIN DEFINITIONS
// ========================

// PWM (must be PWM pins)
const int LF_PWM = 3;
const int LB_PWM = 5;
const int RF_PWM = 6;
const int RB_PWM = 11;

// Direction pins
const int LF_DIR = 2;
const int LB_DIR = 4;
const int RF_DIR = 7;
const int RB_DIR = 8;

// Servo
Servo paintArm;
const int SERVO_PIN = 10;

// ========================
// STATE
// ========================

bool armed = false;
unsigned long lastCommandTime = 0;
const unsigned long TIMEOUT_MS = 500;

// Motor values
int pwm_LF = 0;
int pwm_LB = 0;
int pwm_RF = 0;
int pwm_RB = 0;


// ========================
// SETUP
// ========================

void setup() {
  Serial.begin(9600);

  // Set pins
  pinMode(LF_PWM, OUTPUT);
  pinMode(LB_PWM, OUTPUT);
  pinMode(RF_PWM, OUTPUT);
  pinMode(RB_PWM, OUTPUT);

  pinMode(LF_DIR, OUTPUT);
  pinMode(LB_DIR, OUTPUT);
  pinMode(RF_DIR, OUTPUT);
  pinMode(RB_DIR, OUTPUT);

  // Ensure motors OFF immediately
  analogWrite(LF_PWM, 0);
  analogWrite(LB_PWM, 0);
  analogWrite(RF_PWM, 0);
  analogWrite(RB_PWM, 0);

  // Set forward direction
  digitalWrite(LF_DIR, HIGH);
  digitalWrite(LB_DIR, HIGH);
  digitalWrite(RF_DIR, HIGH);
  digitalWrite(RB_DIR, HIGH);

  // Servo
  paintArm.attach(SERVO_PIN);
  paintArm.write(0);

  stopAll();

  Serial.println("READY");
}


// ========================
// LOOP
// ========================

void loop() {

  // Always enforce stop if not armed
  if (!armed) {
    stopAll();
  }

  // Read serial
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    // ARMING
    if (!armed) {
      if (cmd == "START") {
        armed = true;
        Serial.println("ARMED");
      }
      return;
    }

    // STOP
    if (cmd == "STOP") {
      stopAll();
      armed = false;
      Serial.println("DISARMED");
      return;
    }

    // Parse motor command
    parseAndApply(cmd);
  }

  // Watchdog safety
  if (armed && (millis() - lastCommandTime > TIMEOUT_MS)) {
    stopAll();
    Serial.println("WATCHDOG STOP");
  }

  // Apply motor outputs
  analogWrite(LF_PWM, pwm_LF);
  analogWrite(LB_PWM, pwm_LB);
  analogWrite(RF_PWM, pwm_RF);
  analogWrite(RB_PWM, pwm_RB);
}


// ========================
// PARSER
// ========================

void parseAndApply(String cmd) {

  int l1 = 0, l2 = 0, r1 = 0, r2 = 0, arm = 0;

  sscanf(cmd.c_str(), "L1:%d,L2:%d,R1:%d,R2:%d,ARM:%d",
         &l1, &l2, &r1, &r2, &arm);

  // Clamp values
  l1 = constrain(l1, 0, 255);
  l2 = constrain(l2, 0, 255);
  r1 = constrain(r1, 0, 255);
  r2 = constrain(r2, 0, 255);

  // Assign motors
  pwm_LF = l1;
  pwm_LB = l2;
  pwm_RF = r1;
  pwm_RB = r2;

  // Servo
  paintArm.write(arm ? 90 : 0);

  lastCommandTime = millis();

  // Debug output
  Serial.print("CMD → ");
  Serial.print("L1="); Serial.print(l1);
  Serial.print(" L2="); Serial.print(l2);
  Serial.print(" R1="); Serial.print(r1);
  Serial.print(" R2="); Serial.println(r2);
}


// ========================
// STOP FUNCTION
// ========================

void stopAll() {
  pwm_LF = 0;
  pwm_LB = 0;
  pwm_RF = 0;
  pwm_RB = 0;

  analogWrite(LF_PWM, 0);
  analogWrite(LB_PWM, 0);
  analogWrite(RF_PWM, 0);
  analogWrite(RB_PWM, 0);
}
