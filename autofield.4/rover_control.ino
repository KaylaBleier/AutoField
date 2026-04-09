#include <Servo.h>
// DOES NOT INCLUDE ANY ENCODERS

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
const int SERVO_PIN = 12;
const int SERVO_DOWN = 90;   // adjust these angles to match your setup
const int SERVO_UP   = 0;

void setup() {
  Serial.begin(9600);

  pinMode(PWM_FL, OUTPUT); pinMode(DIR_FL, OUTPUT);
  pinMode(PWM_BL, OUTPUT); pinMode(DIR_BL, OUTPUT);
  pinMode(PWM_FR, OUTPUT); pinMode(DIR_FR, OUTPUT);
  pinMode(PWM_BR, OUTPUT); pinMode(DIR_BR, OUTPUT);

  paintArm.attach(SERVO_PIN);
  paintArm.write(SERVO_UP);

  // All motors off at start
  analogWrite(PWM_FL, 0); analogWrite(PWM_BL, 0);
  analogWrite(PWM_FR, 0); analogWrite(PWM_BR, 0);

  Serial.println("READY");
}

void setLeft(int pwm) {
  digitalWrite(DIR_FL, HIGH);
  digitalWrite(DIR_BL, HIGH);
  analogWrite(PWM_FL, pwm);
  analogWrite(PWM_BL, pwm);
}

void setRight(int pwm) {
  digitalWrite(DIR_FR, HIGH);
  digitalWrite(DIR_BR, HIGH);
  analogWrite(PWM_FR, pwm);
  analogWrite(PWM_BR, pwm);
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    // Handle arming commands from main.py
    if (line == "START") { Serial.println("ARMED"); return; }
    if (line == "STOP")  { setLeft(0); setRight(0); paintArm.write(SERVO_UP); return; }

    // Parse: L1:<v>,L2:<v>,R1:<v>,R2:<v>,ARM:<v>
    int l1 = 0, l2 = 0, r1 = 0, r2 = 0, arm = 0;
    sscanf(line.c_str(), "L1:%d,L2:%d,R1:%d,R2:%d,ARM:%d",
           &l1, &l2, &r1, &r2, &arm);

    setLeft(l1);   // l1 == l2 always, per path_following.py
    setRight(r1);  // r1 == r2 always
    paintArm.write(arm ? SERVO_DOWN : SERVO_UP);
  }
}
