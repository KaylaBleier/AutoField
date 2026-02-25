// ===== Motor Pins =====

// Back wheels
int pwmpin_backleft = 3;
int pwmpin_backright = 5;
int dirpin_backleft = 2;
int dirpin_backright = 4;

// Front wheels
int pwmpin_frontleft = 9;
int pwmpin_frontright = 6;
int dirpin_frontleft = 8;
int dirpin_frontright = 10;

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

void setMotor(int pwmPin, int dirPin, float velocity) {
  int pwm = abs(velocity) * 255;
  pwm = constrain(pwm, 0, 255);

  if (velocity >= 0)
    digitalWrite(dirPin, HIGH);
  else
    digitalWrite(dirPin, LOW);

  analogWrite(pwmPin, pwm);
}

void loop() {

  if (Serial.available()) {

    String line = Serial.readStringUntil('\n');
    line.trim();

    int commaIndex = line.indexOf(',');

    if (commaIndex > 0) {

      float vL = line.substring(0, commaIndex).toFloat();
      float vR = line.substring(commaIndex + 1).toFloat();

      // LEFT side motors
      setMotor(pwmpin_backleft, dirpin_backleft, vL);
      setMotor(pwmpin_frontleft, dirpin_frontleft, vL);

      // RIGHT side motors
      setMotor(pwmpin_backright, dirpin_backright, vR);
      setMotor(pwmpin_frontright, dirpin_frontright, vR);
    }
  }
}
