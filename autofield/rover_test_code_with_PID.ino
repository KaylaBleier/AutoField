#include <Servo.h>
/// Send START to arm
/// Send RPM:150 to set a target (all 4 wheels will PID to that RPM independently)
/// Send L1:200,L2:200,R1:200,R2:200,ARM:0 to manually override and bypass PID
/// Send STOP to disarm

/// PID tuning order — do this with one wheel at a time:

/// Set Ki and Kd to 0, increase Kp until the wheel reaches near target RPM but oscillates
/// Increase Ki slowly until oscillation settles
/// Add a small Kd to reduce overshoot

/// START       ← type this first, hit enter
/// RPM:100     ← then this




// ========================
// HALL SENSOR PINS
// ========================
const int HALL_FL = A0;
const int HALL_BL = A1;
const int HALL_FR = A2;
const int HALL_BR = A3;

// ========================
// MOTOR PINS
// ========================
const int PWM_FL = 5;
const int PWM_BL = 3;
const int PWM_FR = 9;
const int PWM_BR = 6;
const int DIR_FL = 4;
const int DIR_BL = 2;
const int DIR_FR = 8;
const int DIR_BR = 7;

// ========================
// SERVO
// ========================
Servo paintArm;
const int SERVO_PIN = 10;

// ========================
// HALL SENSOR SETTINGS
// ========================
const int RESTING       = 207;
const int THRESHOLD     = 30;
const int NUM_MAGNETS   = 4;
const unsigned long MIN_PULSE_GAP = 30000; // microseconds — tune for your max RPM
const int AVG_SIZE      = 5;

// ========================
// PID SETTINGS
// ========================
float targetRPM = 0;  // Set by command

// Tune these for your motors — start with just Kp, set Ki/Kd to 0 first
const float Kp = 0.5;
const float Ki = 0.05;
const float Kd = 0.01;

// ========================
// PER-WHEEL STATE
// ========================
struct WheelState {
  int     hallPin;
  int     pwmPin;
  int     dirPin;

  // Hall sensing
  bool    lastMagnetState;
  unsigned long lastPulseTime;
  unsigned long periods[AVG_SIZE];
  int     periodIndex;
  int     periodCount;
  float   currentRPM;

  // PID
  float   integral;
  float   lastError;
  int     pwmOutput;
};

WheelState wheels[4];

// ========================
// STATE
// ========================
bool armed = false;
unsigned long lastCommandTime = 0;
const unsigned long TIMEOUT_MS = 5000;

// ========================
// INIT WHEEL
// ========================
void initWheel(WheelState &w, int hall, int pwm, int dir) {
  w.hallPin         = hall;
  w.pwmPin          = pwm;
  w.dirPin          = dir;
  w.lastMagnetState = false;
  w.lastPulseTime   = 0;
  w.periodIndex     = 0;
  w.periodCount     = 0;
  w.currentRPM      = 0;
  w.integral        = 0;
  w.lastError       = 0;
  w.pwmOutput       = 0;
  memset(w.periods, 0, sizeof(w.periods));
}

// ========================
// SETUP
// ========================
void setup() {
  Serial.begin(9600);

  // Init wheel structs
  initWheel(wheels[0], HALL_FL, PWM_FL, DIR_FL);
  initWheel(wheels[1], HALL_BL, PWM_BL, DIR_BL);
  initWheel(wheels[2], HALL_FR, PWM_FR, DIR_FR);
  initWheel(wheels[3], HALL_BR, PWM_BR, DIR_BR);

  // Motor pins
  for (int i = 0; i < 4; i++) {
    pinMode(wheels[i].pwmPin, OUTPUT);
    pinMode(wheels[i].dirPin, OUTPUT);
    digitalWrite(wheels[i].dirPin, HIGH);
  }

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

  // Read and update all hall sensors
  for (int i = 0; i < 4; i++) {
    updateHall(wheels[i]);
  }

  // Run PID and apply output if armed
  if (armed) {
    static unsigned long lastPID = 0;
    if (millis() - lastPID >= 50) {  // Run PID every 50ms
      lastPID = millis();
      for (int i = 0; i < 4; i++) {
        updatePID(wheels[i]);
        analogWrite(wheels[i].pwmPin, wheels[i].pwmOutput);
      }
    }
  }

  // Serial commands
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
      targetRPM = 0;
      Serial.println("DISARMED");
      return;
    }

    parseAndApply(cmd);
  }

  // Watchdog
  if (armed && (millis() - lastCommandTime > TIMEOUT_MS)) {
    stopAll();
    Serial.println("WATCHDOG STOP");
  }

  // Debug print RPM every second
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    Serial.print("Target RPM: "); Serial.print(targetRPM);
    Serial.print(" | FL:"); Serial.print(wheels[0].currentRPM, 1);
    Serial.print(" BL:"); Serial.print(wheels[1].currentRPM, 1);
    Serial.print(" FR:"); Serial.print(wheels[2].currentRPM, 1);
    Serial.print(" BR:"); Serial.println(wheels[3].currentRPM, 1);
  }
}

// ========================
// HALL SENSOR UPDATE
// ========================
void updateHall(WheelState &w) {
  int raw = analogRead(w.hallPin);
  bool magnetDetected = (raw - RESTING) > THRESHOLD;

  if (magnetDetected && !w.lastMagnetState) {
    unsigned long now = micros();
    unsigned long gap = now - w.lastPulseTime;

    if (w.lastPulseTime == 0) {
      w.lastPulseTime = now;
    } else if (gap > MIN_PULSE_GAP) {
      w.periods[w.periodIndex] = gap;
      w.periodIndex = (w.periodIndex + 1) % AVG_SIZE;
      if (w.periodCount < AVG_SIZE) w.periodCount++;
      w.lastPulseTime = now;
    }
  }
  w.lastMagnetState = magnetDetected;

  // Calculate current RPM from averaged periods
  if (w.periodCount == 0 || micros() - w.lastPulseTime > 1000000) {
    w.currentRPM = 0;
  } else {
    unsigned long avgPeriod = 0;
    for (int i = 0; i < w.periodCount; i++) avgPeriod += w.periods[i];
    avgPeriod /= w.periodCount;
    w.currentRPM = (60.0 * 1000000.0) / (avgPeriod * NUM_MAGNETS);
  }
}

// ========================
// PID UPDATE
// ========================
void updatePID(WheelState &w) {
  if (targetRPM == 0) {
    w.integral  = 0;
    w.lastError = 0;
    w.pwmOutput = 0;
    return;
  }

  float error = targetRPM - w.currentRPM;
  w.integral  += error * 0.05;                    // 0.05 = PID interval in seconds
  w.integral   = constrain(w.integral, -255, 255); // Prevent windup
  float derivative = (error - w.lastError) / 0.05;
  w.lastError  = error;

  float output = (Kp * error) + (Ki * w.integral) + (Kd * derivative);
  w.pwmOutput  = constrain((int)output, 0, 255);
}

// ========================
// PARSER
// ========================
void parseAndApply(String cmd) {
  // Set target RPM: "RPM:150"
  if (cmd.startsWith("RPM:")) {
    targetRPM = cmd.substring(4).toFloat();
    // Reset PID state on new target
    for (int i = 0; i < 4; i++) {
      wheels[i].integral  = 0;
      wheels[i].lastError = 0;
    }
    Serial.print("Target RPM set to: ");
    Serial.println(targetRPM);
    lastCommandTime = millis();
    return;
  }

  // Manual override: "L1:200,L2:200,R1:200,R2:200,ARM:0"
  // (bypasses PID, sets PWM directly — useful for testing)
  int l1 = 0, l2 = 0, r1 = 0, r2 = 0, arm = 0;
  if (sscanf(cmd.c_str(), "L1:%d,L2:%d,R1:%d,R2:%d,ARM:%d",
             &l1, &l2, &r1, &r2, &arm) == 5) {
    targetRPM = 0;  // Disable PID when manually overriding
    wheels[0].pwmOutput = constrain(l1, 0, 255);
    wheels[1].pwmOutput = constrain(l2, 0, 255);
    wheels[2].pwmOutput = constrain(r1, 0, 255);
    wheels[3].pwmOutput = constrain(r2, 0, 255);
    for (int i = 0; i < 4; i++) {
      analogWrite(wheels[i].pwmPin, wheels[i].pwmOutput);
    }
    paintArm.write(arm ? 90 : 0);
    lastCommandTime = millis();
    Serial.print("Manual PWM → FL:"); Serial.print(l1);
    Serial.print(" BL:"); Serial.print(l2);
    Serial.print(" FR:"); Serial.print(r1);
    Serial.print(" BR:"); Serial.println(r2);
  }
}

// ========================
// STOP
// ========================
void stopAll() {
  targetRPM = 0;
  for (int i = 0; i < 4; i++) {
    wheels[i].pwmOutput = 0;
    wheels[i].integral  = 0;
    wheels[i].lastError = 0;
    analogWrite(wheels[i].pwmPin, 0);
  }
}