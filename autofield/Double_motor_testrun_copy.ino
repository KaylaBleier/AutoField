const int potpin = A0; //analog pin used to connect the center pin of potentiometer

int pwmpin_l = 3;
int pwmpin_r = 5;
int dirpin_l = 2;
int dirpin_r = 4;

int pwmpin_val;


void setup() {
  // put your setup code here, to run once:

  pinMode(pwmpin_l, OUTPUT);
  pinMode(pwmpin_r, OUTPUT);
  pinMode(dirpin_l, OUTPUT);
  pinMode(dirpin_r, OUTPUT);
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  pwmpin_val = analogRead(potpin);
  pwmpin_val = map(pwmpin_val, 0, 1023, 0, 255);
  digitalWrite(dirpin_l, HIGH);
  digitalWrite(dirpin_r, HIGH);
  analogWrite(pwmpin_l, pwmpin_val);
  analogWrite(pwmpin_r, pwmpin_val);
  Serial.print("Motor speed pwm command: ");
  Serial.print(pwmpin_val);
  Serial.println();
}
