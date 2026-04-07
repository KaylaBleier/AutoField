// 4-WHEEL RPM MEASUREMENT + PROPORTIONAL CONTROL (P-ONLY)
// 1 Magnet Pass = 1 Revolution
//tuesday april 7, v3


// ------------------------------------------------------------
// MOTOR PIN DEFINITIONS
// ------------------------------------------------------------

// Back wheels
int pwmpin_backleft   = 3;
int pwmpin_backright  = 6;
int dirpin_backleft   = 2;
int dirpin_backright  = 7;

// Front wheels
int pwmpin_frontleft  = 5;
int pwmpin_frontright = 9;
int dirpin_frontleft  = 4;
int dirpin_frontright = 8;


// ------------------------------------------------------------
// HALL SENSOR PINS
// ------------------------------------------------------------

const int hallAnalogPin_BL = A5;
const int hallAnalogPin_FL = A0;
const int hallAnalogPin_FR = A1;
const int hallAnalogPin_BR = A4;


// ------------------------------------------------------------
// TARGET RPM
// ------------------------------------------------------------

float targetRPM = 45.0;

// ------------------------------------------------------------
// PER-WHEEL PROPORTIONAL GAINS
// Start all equal, then adjust individually if needed
// ------------------------------------------------------------

float Kp_BL = 1.5;
float Kp_FL = 1.5;
float Kp_FR = 1.5;
float Kp_BR = 1.5;


// PWM outputs
int pwm_BL = 0;
int pwm_FL = 0;
int pwm_FR = 0;
int pwm_BR = 0;


// ------------------------------------------------------------
// BACK LEFT VARIABLES
// ------------------------------------------------------------
int hallAnalog_BL;
unsigned long passCount_BL = 0;
bool magnetPresent_BL = false;
unsigned long lastPassTime_BL = 0;
unsigned long timeBetweenPasses_BL = 0;
float rpm_BL = 0;


// ------------------------------------------------------------
// FRONT LEFT VARIABLES
// ------------------------------------------------------------
int hallAnalog_FL;
unsigned long passCount_FL = 0;
bool magnetPresent_FL = false;
unsigned long lastPassTime_FL = 0;
unsigned long timeBetweenPasses_FL = 0;
float rpm_FL = 0;


// ------------------------------------------------------------
// FRONT RIGHT VARIABLES
// ------------------------------------------------------------
int hallAnalog_FR;
unsigned long passCount_FR = 0;
bool magnetPresent_FR = false;
unsigned long lastPassTime_FR = 0;
unsigned long timeBetweenPasses_FR = 0;
float rpm_FR = 0;


// ------------------------------------------------------------
// BACK RIGHT VARIABLES
// ------------------------------------------------------------
int hallAnalog_BR;
unsigned long passCount_BR = 0;
bool magnetPresent_BR = false;
unsigned long lastPassTime_BR = 0;
unsigned long timeBetweenPasses_BR = 0;
float rpm_BR = 0;


// ============================================================
// SETUP
// ============================================================

void setup() {

  pinMode(pwmpin_backleft, OUTPUT);
  pinMode(pwmpin_backright, OUTPUT);
  pinMode(dirpin_backleft, OUTPUT);
  pinMode(dirpin_backright, OUTPUT);

  pinMode(pwmpin_frontleft, OUTPUT);
  pinMode(pwmpin_frontright, OUTPUT);
  pinMode(dirpin_frontleft, OUTPUT);
  pinMode(dirpin_frontright, OUTPUT);

  Serial.begin(9600);
}


// ============================================================
// LOOP
// ============================================================

void loop() {

  // ------------------------------------------------------------
  // SET MOTOR DIRECTION (FORWARD)
  // ------------------------------------------------------------

  digitalWrite(dirpin_backleft, HIGH);
  digitalWrite(dirpin_backright, HIGH);
  digitalWrite(dirpin_frontleft, HIGH);
  digitalWrite(dirpin_frontright, HIGH);


  // ============================================================
  // BACK LEFT RPM MEASUREMENT
  // ============================================================

  hallAnalog_BL = analogRead(hallAnalogPin_BL);
  bool magnetDetected_BL = (hallAnalog_BL >= 200 && hallAnalog_BL <= 290);

  static bool firstLoop_BL = true;
  if (firstLoop_BL) {
    magnetPresent_BL = magnetDetected_BL;
    lastPassTime_BL = millis();
    firstLoop_BL = false;
  }

  if (magnetDetected_BL && !magnetPresent_BL) {
    unsigned long now = millis();
    timeBetweenPasses_BL = now - lastPassTime_BL;
    lastPassTime_BL = now;

    if (timeBetweenPasses_BL > 0)
      rpm_BL = 15000.0 / timeBetweenPasses_BL;

    passCount_BL++;
    magnetPresent_BL = true;
  }

  if (!magnetDetected_BL && magnetPresent_BL)
    magnetPresent_BL = false;


  // ============================================================
  // FRONT LEFT RPM MEASUREMENT
  // ============================================================

  hallAnalog_FL = analogRead(hallAnalogPin_FL);
  bool magnetDetected_FL = (hallAnalog_FL >= 200 && hallAnalog_FL <= 290);

  static bool firstLoop_FL = true;
  if (firstLoop_FL) {
    magnetPresent_FL = magnetDetected_FL;
    lastPassTime_FL = millis();
    firstLoop_FL = false;
  }

  if (magnetDetected_FL && !magnetPresent_FL) {
    unsigned long now = millis();
    timeBetweenPasses_FL = now - lastPassTime_FL;
    lastPassTime_FL = now;

    if (timeBetweenPasses_FL > 0)
      rpm_FL = 15000.0 / timeBetweenPasses_FL;

    passCount_FL++;
    magnetPresent_FL = true;
  }

  if (!magnetDetected_FL && magnetPresent_FL)
    magnetPresent_FL = false;


  // ============================================================
  // FRONT RIGHT RPM MEASUREMENT
  // ============================================================

  hallAnalog_FR = analogRead(hallAnalogPin_FR);
  bool magnetDetected_FR = (hallAnalog_FR >= 200 && hallAnalog_FR <= 290);

  static bool firstLoop_FR = true;
  if (firstLoop_FR) {
    magnetPresent_FR = magnetDetected_FR;
    lastPassTime_FR = millis();
    firstLoop_FR = false;
  }

  if (magnetDetected_FR && !magnetPresent_FR) {
    unsigned long now = millis();
    timeBetweenPasses_FR = now - lastPassTime_FR;
    lastPassTime_FR = now;

    if (timeBetweenPasses_FR > 0)
      rpm_FR = 15000.0 / timeBetweenPasses_FR;

    passCount_FR++;
    magnetPresent_FR = true;
  }

  if (!magnetDetected_FR && magnetPresent_FR)
    magnetPresent_FR = false;


  // ============================================================
  // BACK RIGHT RPM MEASUREMENT
  // ============================================================

  hallAnalog_BR = analogRead(hallAnalogPin_BR);
  bool magnetDetected_BR = (hallAnalog_BR >= 200 && hallAnalog_BR <= 290);

  static bool firstLoop_BR = true;
  if (firstLoop_BR) {
    magnetPresent_BR = magnetDetected_BR;
    lastPassTime_BR = millis();
    firstLoop_BR = false;
  }

  if (magnetDetected_BR && !magnetPresent_BR) {
    unsigned long now = millis();
    timeBetweenPasses_BR = now - lastPassTime_BR;
    lastPassTime_BR = now;

    if (timeBetweenPasses_BR > 0)
      rpm_BR = 15000.0 / timeBetweenPasses_BR;

    passCount_BR++;
    magnetPresent_BR = true;
  }

  if (!magnetDetected_BR && magnetPresent_BR)
    magnetPresent_BR = false;


  // ============================================================
  // PROPORTIONAL CONTROL (PER-WHEEL Kp)
  // ============================================================

  float error_BL = targetRPM - rpm_BL;
  pwm_BL = constrain(Kp_BL * error_BL, 0, 255);
  analogWrite(pwmpin_backleft, pwm_BL);

  float error_FL = targetRPM - rpm_FL;
  pwm_FL = constrain(Kp_FL * error_FL, 0, 255);
  analogWrite(pwmpin_frontleft, pwm_FL);

  float error_FR = targetRPM - rpm_FR;
  pwm_FR = constrain(Kp_FR * error_FR, 0, 255);
  analogWrite(pwmpin_frontright, pwm_FR);

  float error_BR = targetRPM - rpm_BR;
  pwm_BR = constrain(Kp_BR * error_BR, 0, 255);
  analogWrite(pwmpin_backright, pwm_BR);


  // ------------------------------------------------------------
  // SERIAL OUTPUT
  // ------------------------------------------------------------

  Serial.print("T:");
  Serial.print(targetRPM);

  Serial.print(" | BL:");
  Serial.print(rpm_BL, 1);

  Serial.print("KP_Error:");
  Serial.print(error_BL, 1);


  Serial.print(" | FL:");
  Serial.print(rpm_FL, 1);

  Serial.print(" | FR:");
  Serial.print(rpm_FR, 1);

  Serial.print(" | BR:");
  Serial.print(rpm_BR, 1);

  Serial.println();

  delay(10);
}
