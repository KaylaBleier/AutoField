#include <Servo.h>

// ========================
// CORRECT PIN DEFINITIONS
// ========================

// LEFT SIDE
const int PWM_FL = 5;
const int PWM_BL = 3;
const int DIR_FL = 4;
const int DIR_BL = 2;

// RIGHT SIDE
const int PWM_FR = 9;
const int PWM_BR = 6;
const int DIR_FR = 8;
const int DIR_BR = 7;

// Servo
Servo paintArm;
const int SERVO_PIN = 10;

// ========================
// STATE
// ========================

bool armed = false;
unsigned long lastCommandTime = 0;
const unsigned long TIMEOUT_MS = 5000;

// PWM values
int pwm_FL = 0;
int pwm_BL = 0;
int pwm_FR = 0;
int pwm_BR = 0;


// ========================
// SETUP
// ========================

void setup() {
  Serial.begin(9600);

  pinMode(PWM_FL, OUTPUT);
  pinMode(PWM_BL, OUTPUT);
  pinMode(PWM_FR, OUTPUT);
  pinMode(PWM_BR, OUTPUT);

  pinMode(DIR_FL, OUTPUT);
  pinMode(DIR_BL, OUTPUT);
  pinMode(DIR_FR, OUTPUT);
  pinMode(DIR_BR, OUTPUT);

  // Set forward direction
  digitalWrite(DIR_FL, HIGH);
  digitalWrite(DIR_BL, HIGH);
  digitalWrite(DIR_FR, HIGH);
  digitalWrite(DIR_BR, HIGH);

  stopAll();

  paintArm.attach(SERVO_PIN);
  paintArm.write(0);

  Serial.println("READY");
}


// ========================
// LOOP
// ========================

void loop() {

  if (!armed) {
    stopAll();
  }

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (!armed) {
      if (cmd == "START") {
        armed = true;
        Serial.println("ARMED");
      }
      return;
    }

    if (cmd == "STOP") {
      stopAll();
      armed = false;
      Serial.println("DISARMED");
      return;
    }

    parseAndApply(cmd);
  }

  if (armed && (millis() - lastCommandTime > TIMEOUT_MS)) {
    stopAll();
    Serial.println("WATCHDOG STOP");
  }

  analogWrite(PWM_FL, pwm_FL);
  analogWrite(PWM_BL, pwm_BL);
  analogWrite(PWM_FR, pwm_FR);
  analogWrite(PWM_BR, pwm_BR);
}


// ========================
// PARSER
// ========================

void parseAndApply(String cmd) {

  int l1 = 0, l2 = 0, r1 = 0, r2 = 0, arm = 0;

  sscanf(cmd.c_str(), "L1:%d,L2:%d,R1:%d,R2:%d,ARM:%d",
         &l1, &l2, &r1, &r2, &arm);

  l1 = constrain(l1, 0, 255);
  l2 = constrain(l2, 0, 255);
  r1 = constrain(r1, 0, 255);
  r2 = constrain(r2, 0, 255);

  // CORRECT MAPPING
  pwm_FL = l1;
  pwm_BL = l2;
  pwm_FR = r1;
  pwm_BR = r2;

  paintArm.write(arm ? 90 : 0);

  lastCommandTime = millis();

  Serial.print("PWM → ");
  Serial.print("FL:"); Serial.print(l1);
  Serial.print(" BL:"); Serial.print(l2);
  Serial.print(" FR:"); Serial.print(r1);
  Serial.print(" BR:"); Serial.println(r2);
}


// ========================
// STOP
// ========================

void stopAll() {
  pwm_FL = 0;
  pwm_BL = 0;
  pwm_FR = 0;
  pwm_BR = 0;

  analogWrite(PWM_FL, 0);
  analogWrite(PWM_BL, 0);
  analogWrite(PWM_FR, 0);
  analogWrite(PWM_BR, 0);
}
