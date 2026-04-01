// ------------------------------------------------------------
// BACK LEFT WHEEL (Wheel 1)
// ------------------------------------------------------------
const int hallAnalogPin_BL = A5;
const int LED_BL = 8;

int hallAnalog_BL;
unsigned long passCount_BL = 0;
bool magnetPresent_BL = false;
unsigned long lastPassTime_BL = 0;
unsigned long timeBetweenPasses_BL = 0;
float rpm_BL = 0;


// ------------------------------------------------------------
// FRONT LEFT WHEEL (Wheel 2)
// ------------------------------------------------------------
const int hallAnalogPin_FL = A0;
const int LED_FL = 9;

int hallAnalog_FL;
unsigned long passCount_FL = 0;
bool magnetPresent_FL = false;
unsigned long lastPassTime_FL = 0;
unsigned long timeBetweenPasses_FL = 0;
float rpm_FL = 0;


// ------------------------------------------------------------
// FRONT RIGHT WHEEL (Wheel 3)
// ------------------------------------------------------------
const int hallAnalogPin_FR = A1;
const int LED_FR = 10;

int hallAnalog_FR;
unsigned long passCount_FR = 0;
bool magnetPresent_FR = false;
unsigned long lastPassTime_FR = 0;
unsigned long timeBetweenPasses_FR = 0;
float rpm_FR = 0;


// ------------------------------------------------------------
// BACK RIGHT WHEEL (Wheel 4)
// ------------------------------------------------------------
const int hallAnalogPin_BR = A4;
const int LED_BR = 11;

int hallAnalog_BR;
unsigned long passCount_BR = 0;
bool magnetPresent_BR = false;
unsigned long lastPassTime_BR = 0;
unsigned long timeBetweenPasses_BR = 0;
float rpm_BR = 0;


// ------------------------------------------------------------
// PID CONTROL VARIABLES (P ONLY ACTIVE — I & D COMMENTED OUT)
// ------------------------------------------------------------
float targetRPM = 120.0;

float Kp = 1.0;
// float Ki = 0.00;   // <--- INTEGRAL (disabled for now)
// float Kd = 0.00;   // <--- DERIVATIVE (disabled for now)

int PWM_BL = 3;
int PWM_FL = 5;
int PWM_FR = 6;
int PWM_BR = 9;

int pwm_BL = 0;
int pwm_FL = 0;
int pwm_FR = 0;
int pwm_BR = 0;

// Integral accumulators (commented out)
/*
float integral_BL = 0;
float integral_FL = 0;
float integral_FR = 0;
float integral_BR = 0;
*/

// Last errors for derivative (commented out)
/*
float lastError_BL = 0;
float lastError_FL = 0;
float lastError_FR = 0;
float lastError_BR = 0;
*/


// ------------------------------------------------------------
// SETUP
// ------------------------------------------------------------
void setup() {
  pinMode(LED_BL, OUTPUT);
  pinMode(LED_FL, OUTPUT);
  pinMode(LED_FR, OUTPUT);
  pinMode(LED_BR, OUTPUT);

  pinMode(PWM_BL, OUTPUT);
  pinMode(PWM_FL, OUTPUT);
  pinMode(PWM_FR, OUTPUT);
  pinMode(PWM_BR, OUTPUT);

  Serial.begin(9600);
}


// ------------------------------------------------------------
// LOOP
// ------------------------------------------------------------
void loop() {

  // ============================
  // WHEEL 1: BACK LEFT (BL)
  // ============================
  hallAnalog_BL = analogRead(hallAnalogPin_BL);
  bool magnetDetected_BL = (hallAnalog_BL >= 220 && hallAnalog_BL <= 250);

  static bool firstLoop_BL = true;
  if (firstLoop_BL) { magnetPresent_BL = magnetDetected_BL; lastPassTime_BL = millis(); firstLoop_BL = false; }

  digitalWrite(LED_BL, magnetDetected_BL ? HIGH : LOW);

  if (magnetDetected_BL && !magnetPresent_BL) {
    unsigned long now = millis();
    timeBetweenPasses_BL = now - lastPassTime_BL;
    lastPassTime_BL = now;
    if (timeBetweenPasses_BL > 0) rpm_BL = 60000.0 / timeBetweenPasses_BL;
    passCount_BL++;
    magnetPresent_BL = true;
  }
  if (!magnetDetected_BL && magnetPresent_BL) magnetPresent_BL = false;


  // ============================
  // WHEEL 2: FRONT LEFT (FL)
  // ============================
  hallAnalog_FL = analogRead(hallAnalogPin_FL);
  bool magnetDetected_FL = (hallAnalog_FL >= 220 && hallAnalog_FL <= 250);

  static bool firstLoop_FL = true;
  if (firstLoop_FL) { magnetPresent_FL = magnetDetected_FL; lastPassTime_FL = millis(); firstLoop_FL = false; }

  digitalWrite(LED_FL, magnetDetected_FL ? HIGH : LOW);

  if (magnetDetected_FL && !magnetPresent_FL) {
    unsigned long now = millis();
    timeBetweenPasses_FL = now - lastPassTime_FL;
    lastPassTime_FL = now;
    if (timeBetweenPasses_FL > 0) rpm_FL = 60000.0 / timeBetweenPasses_FL;
    passCount_FL++;
    magnetPresent_FL = true;
  }
  if (!magnetDetected_FL && magnetPresent_FL) magnetPresent_FL = false;


  // ============================
  // WHEEL 3: FRONT RIGHT (FR)
  // ============================
  hallAnalog_FR = analogRead(hallAnalogPin_FR);
  bool magnetDetected_FR = (hallAnalog_FR >= 220 && hallAnalog_FR <= 250);

  static bool firstLoop_FR = true;
  if (firstLoop_FR) { magnetPresent_FR = magnetDetected_FR; lastPassTime_FR = millis(); firstLoop_FR = false; }

  digitalWrite(LED_FR, magnetDetected_FR ? HIGH : LOW);

  if (magnetDetected_FR && !magnetPresent_FR) {
    unsigned long now = millis();
    timeBetweenPasses_FR = now - lastPassTime_FR;
    lastPassTime_FR = now;
    if (timeBetweenPasses_FR > 0) rpm_FR = 60000.0 / timeBetweenPasses_FR;
    passCount_FR++;
    magnetPresent_FR = true;
  }
  if (!magnetDetected_FR && magnetPresent_FR) magnetPresent_FR = false;


  // ============================
  // WHEEL 4: BACK RIGHT (BR)
  // ============================
  hallAnalog_BR = analogRead(hallAnalogPin_BR);
  bool magnetDetected_BR = (hallAnalog_BR >= 220 && hallAnalog_BR <= 250);

  static bool firstLoop_BR = true;
  if (firstLoop_BR) { magnetPresent_BR = magnetDetected_BR; lastPassTime_BR = millis(); firstLoop_BR = false; }

  digitalWrite(LED_BR, magnetDetected_BR ? HIGH : LOW);

  if (magnetDetected_BR && !magnetPresent_BR) {
    unsigned long now = millis();
    timeBetweenPasses_BR = now - lastPassTime_BR;
    lastPassTime_BR = now;
    if (timeBetweenPasses_BR > 0) rpm_BR = 60000.0 / timeBetweenPasses_BR;
    passCount_BR++;
    magnetPresent_BR = true;
  }
  if (!magnetDetected_BR && magnetPresent_BR) magnetPresent_BR = false;


  // ------------------------------------------------------------
  // SERIAL OUTPUT (4 wheels on one line)
  // ------------------------------------------------------------
  Serial.print("BL A:");
  Serial.print(hallAnalog_BL);
  Serial.print(" C:");
  Serial.print(passCount_BL);
  Serial.print(" RPM:");
  Serial.print(rpm_BL, 1);

  Serial.print(" | FL A:");
  Serial.print(hallAnalog_FL);
  Serial.print(" C:");
  Serial.print(passCount_FL);
  Serial.print(" RPM:");
  Serial.print(rpm_FL, 1);

  Serial.print(" | FR A:");
  Serial.print(hallAnalog_FR);
  Serial.print(" C:");
  Serial.print(passCount_FR);
  Serial.print(" RPM:");
  Serial.print(rpm_FR, 1);

  Serial.print(" | BR A:");
  Serial.print(hallAnalog_BR);
  Serial.print(" C:");
  Serial.print(passCount_BR);
  Serial.print(" RPM:");
  Serial.println(rpm_BR, 1);


  // ------------------------------------------------------------
  // PROPORTIONAL MOTOR CONTROL (I & D COMMENTED OUT)
  // ------------------------------------------------------------

  float error_BL = targetRPM - rpm_BL;
  float error_FL = targetRPM - rpm_FL;
  float error_FR = targetRPM - rpm_FR;
  float error_BR = targetRPM - rpm_BR;

  // ---- Integral (disabled) ----
  /*
  integral_BL += error_BL;
  integral_FL += error_FL;
  integral_FR += error_FR;
  integral_BR += error_BR;
  */

  // ---- Derivative (disabled) ----
  /*
  float derivative_BL = error_BL - lastError_BL;
  float derivative_FL = error_FL - lastError_FL;
  float derivative_FR = error_FR - lastError_FR;
  float derivative_BR = error_BR - lastError_BR;
  */

  // ---- PID output (P only active) ----
  float control_BL = Kp * error_BL; // + Ki * integral_BL + Kd * derivative_BL;
  float control_FL = Kp * error_FL; // + Ki * integral_FL + Kd * derivative_FL;
  float control_FR = Kp * error_FR; // + Ki * integral_FR + Kd * derivative_FR;
  float control_BR = Kp * error_BR; // + Ki * integral_BR + Kd * derivative_BR;

  // ---- Update PWM ----
  pwm_BL += control_BL;
  pwm_FL += control_FL;
  pwm_FR += control_FR;
  pwm_BR += control_BR;

  // ---- Clamp ----
  pwm_BL = constrain(pwm_BL, 0, 255);
  pwm_FL = constrain(pwm_FL, 0, 255);
  pwm_FR = constrain(pwm_FR, 0, 255);
  pwm_BR = constrain(pwm_BR, 0, 255);

  // ---- Write to motors ----
  analogWrite(PWM_BL, pwm_BL);
  analogWrite(PWM_FL, pwm_FL);
  analogWrite(PWM_FR, pwm_FR);
  analogWrite(PWM_BR, pwm_BR);

  // ---- Save last errors for derivative (commented) ----
  /*
  lastError_BL = error_BL;
  lastError_FL = error_FL;
  lastError_FR = error_FR;
  lastError_BR = error_BR;
  */

  delay(10);
}
