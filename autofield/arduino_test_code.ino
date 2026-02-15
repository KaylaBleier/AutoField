#define PWM_PIN 5
#define DIR_PIN 4

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
}

void loop() {

  // Forward slow
  digitalWrite(DIR_PIN, HIGH);
  analogWrite(PWM_PIN, 100);
  delay(3000);

  // Stop
  analogWrite(PWM_PIN, 0);
  delay(2000);

  // Reverse slow
  digitalWrite(DIR_PIN, LOW);
  analogWrite(PWM_PIN, 100);
  delay(3000);

  // Stop
  analogWrite(PWM_PIN, 0);
  delay(4000);
}
