// ============================================================
//  Skid-Steer Rover — Geometry-Based Dead Reckoning
//  Rectangle Tracing via Serial Input
//  Target: Arduino Uno/Nano  |  Driver: Cytron 30A (PWM + DIR)
//  Open-loop (PWM + time), encoder-ready hooks included
// ============================================================
//
//  ROVER GEOMETRY (update if hardware changes)
//    Wheel diameter  : 7.198  in
//    Wheelbase       : 15.95  in  (front-to-rear axle)
//    Track width     : 24.908 in  (left-to-right wheel center)
//    Linear speed    : 23.2   in/s @ PWM 100 (measured)
//    Reverse dist    : 33.5   in  (back-and-fill after each turn)
//
//  WIRING (Cytron 30A — PWM + DIR per motor)
//    Front Left  : PWM=5,  DIR=4
//    Rear  Left  : PWM=3,  DIR=2
//    Front Right : PWM=9,  DIR=8
//    Rear  Right : PWM=6,  DIR=7
//
//  DIR pin: HIGH = forward, LOW = reverse
//
//  SERIAL USAGE
//    Open Serial Monitor @ 9600 baud
//    Enter:  <length>,<width>   e.g.  48,24
//    Units match whatever units you calibrated speed in (inches)
// ============================================================

// ── Pin Definitions ─────────────────────────────────────────
// Left side
const int PWM_FL = 5;
const int DIR_FL = 4;
const int PWM_BL = 3;
const int DIR_BL = 2;

// Right side
const int PWM_FR = 9;
const int DIR_FR = 8;
const int PWM_BR = 6;
const int DIR_BR = 7;

// ── Rover Constants ──────────────────────────────────────────
const float WHEEL_DIAMETER_IN = 7.198;
const float WHEEL_BASE_IN     = 15.95;
const float TRACK_WIDTH_IN    = 24.908;
const float LINEAR_SPEED_IPS  = 23.2;   // inches per second @ PWM 100
const int   DRIVE_PWM         = 100;

// Derived
const float WHEEL_CIRC_IN     = PI * WHEEL_DIAMETER_IN;  // ~22.613"
const float TURN_RADIUS_IN    = TRACK_WIDTH_IN / 2.0;    // pivot: inner stops

// Pause between maneuvers (ms) — lets rover settle
const float REVERSE_DIST_IN   = 33.5;   // back-and-fill reverse distance
const int   SETTLE_MS         = 300;

// ── Serial Input Buffer ──────────────────────────────────────
String inputBuffer = "";

// ── Prototypes ───────────────────────────────────────────────
void driveLeft(int pwm);
void driveRight(int pwm);
void driveLeftReverse(int pwm);
void driveRightReverse(int pwm);
void stopAll();
void moveForward(float distanceIn);
void moveBackward(float distanceIn);
void pivotRight90();
void traceRectangle(float lengthIn, float widthIn);
float distanceToTime(float distanceIn);

// ============================================================
void setup() {
  Serial.begin(9600);

  // Set all motor pins as output
  int pins[] = {PWM_FL, DIR_FL, PWM_BL, DIR_BL,
                PWM_FR, DIR_FR, PWM_BR, DIR_BR};
  for (int i = 0; i < 8; i++) {
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], LOW);
  }

  Serial.println(F("=== Skid-Steer Rectangle Rover ==="));
  Serial.println(F("Enter rectangle dimensions as:  length,width"));
  Serial.println(F("Example:  48,24  (inches)"));
  Serial.println(F("Waiting for input..."));
}

// ============================================================
void loop() {
  // Collect characters until newline
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        parseAndRun(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }
}

// ── Parse "length,width" and launch rectangle trace ─────────
void parseAndRun(String s) {
  s.trim();
  int commaIdx = s.indexOf(',');
  if (commaIdx < 0) {
    Serial.println(F("ERROR: Use format  length,width  (e.g. 48,24)"));
    return;
  }

  float len = s.substring(0, commaIdx).toFloat();
  float wid = s.substring(commaIdx + 1).toFloat();

  if (len <= 0 || wid <= 0) {
    Serial.println(F("ERROR: Dimensions must be positive numbers."));
    return;
  }

  Serial.print(F("Tracing rectangle: "));
  Serial.print(len); Serial.print(F("\" x "));
  Serial.print(wid); Serial.println(F("\""));

  traceRectangle(len, wid);

  Serial.println(F("Done! Enter next dimensions or same again."));
}

// ============================================================
//  HIGH-LEVEL MANEUVERS
// ============================================================

// Trace a rectangle: 2× length sides, 2× width sides, 4 right turns
// After each turn: reverse REVERSE_DIST_IN, then drive full next side
void traceRectangle(float lengthIn, float widthIn) {
  for (int side = 0; side < 4; side++) {
    float dist = (side % 2 == 0) ? lengthIn : widthIn;

    Serial.print(F("  Driving side "));
    Serial.print(side + 1);
    Serial.print(F("  →  "));
    Serial.print(dist);
    Serial.println(F("\""));

    moveForward(dist);
    delay(SETTLE_MS);

    Serial.println(F("  Turning right 90°"));
    pivotRight90();
    delay(SETTLE_MS);

    Serial.print(F("  Reversing "));
    Serial.print(REVERSE_DIST_IN);
    Serial.println(F("\""));
    moveBackward(REVERSE_DIST_IN);
    delay(SETTLE_MS);
  }

  stopAll();
}

// ── Drive straight for a given distance ─────────────────────
void moveForward(float distanceIn) {
  unsigned long tMs = (unsigned long)(distanceToTime(distanceIn) * 1000.0);
  driveLeft(DRIVE_PWM);
  driveRight(DRIVE_PWM);
  delay(tMs);
  stopAll();
}

// ── Drive straight backward for a given distance ─────────────
void moveBackward(float distanceIn) {
  unsigned long tMs = (unsigned long)(distanceToTime(distanceIn) * 1000.0);
  driveLeftReverse(DRIVE_PWM);
  driveRightReverse(DRIVE_PWM);
  delay(tMs);
  stopAll();
}

 (inner/right side stops, left side drives)
//    Arc length outer wheel = (PI/2) * TRACK_WIDTH_IN
void pivotRight90() {
  float arcLength = (PI / 2.0) * TRACK_WIDTH_IN;  // ~19.57"
  unsigned long tMs = (unsigned long)(distanceToTime(arcLength) * 1000.0);

  driveLeft(DRIVE_PWM);   // outer (left) drives
  driveRight(0);           // inner (right) stops  ← swap for left turn
  delay(tMs);
  stopAll();
}

// ── Convert inches → seconds at current speed ───────────────
float distanceToTime(float distanceIn) {
  return distanceIn / LINEAR_SPEED_IPS;
}

// ============================================================
//  MOTOR PRIMITIVES  (Cytron 30A — PWM + DIR)
//  DIR HIGH = forward, DIR LOW = reverse
// ============================================================

void driveLeft(int pwm) {
  digitalWrite(DIR_FL, HIGH);  analogWrite(PWM_FL, pwm);
  digitalWrite(DIR_BL, HIGH);  analogWrite(PWM_BL, pwm);
}

void driveRight(int pwm) {
  digitalWrite(DIR_FR, HIGH);  analogWrite(PWM_FR, pwm);
  digitalWrite(DIR_BR, HIGH);  analogWrite(PWM_BR, pwm);
}

void driveLeftReverse(int pwm) {
  digitalWrite(DIR_FL, LOW);  analogWrite(PWM_FL, pwm);
  digitalWrite(DIR_BL, LOW);  analogWrite(PWM_BL, pwm);
}

void driveRightReverse(int pwm) {
  digitalWrite(DIR_FR, LOW);  analogWrite(PWM_FR, pwm);
  digitalWrite(DIR_BR, LOW);  analogWrite(PWM_BR, pwm);
}

void stopAll() {
  analogWrite(PWM_FL, 0);  analogWrite(PWM_BL, 0);
  analogWrite(PWM_FR, 0);  analogWrite(PWM_BR, 0);
}

// ============================================================
//  ENCODER HOOK (placeholder — wire in when encoders are added)
// ============================================================
//
//  1. Attach interrupts to encoder pins, increment pulse counters.
//  2. Replace distanceToTime() calls with pulse-count targets:
//
//       float pulsesPerInch = ENCODER_PPR / WHEEL_CIRC_IN;
//       long targetPulses   = (long)(distanceIn * pulsesPerInch);
//       while (leftPulses < targetPulses) { /* drive */ }
//       stopAll();
//
//  3. For turns, compute each side's arc length separately and
//     run each side to its individual pulse target.
//
// ============================================================
