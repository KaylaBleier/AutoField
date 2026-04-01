const int potpin = A0; //analog pin used to connect the center pin of potentiometer

// connections to arduino, back wheels (pwm white, orange dir)
int pwmpin_backleft = 3;
int pwmpin_backright = 7;
int dirpin_backleft = 2;
int dirpin_backright = 6;

// arduino connections, front wheels
int pwmpin_frontleft = 5;
int pwmpin_frontright = 9;
int dirpin_frontleft = 4;
int dirpin_frontright = 8;


int pwmpin_val;


void setup() {
  // put your setup code here, to run once:

 // back wheel pwm and direction outputs
  pinMode(pwmpin_backleft, OUTPUT);
  pinMode(pwmpin_backright, OUTPUT);

  pinMode(dirpin_backleft, OUTPUT);
  pinMode(dirpin_backright, OUTPUT);

 // ADD front wheel pwm and direction outputs
  pinMode(pwmpin_frontleft, OUTPUT);
  pinMode(pwmpin_frontright, OUTPUT);

  pinMode(dirpin_frontleft, OUTPUT);
  pinMode(dirpin_frontright, OUTPUT);

  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  pwmpin_val = analogRead(potpin);
  pwmpin_val = map(pwmpin_val, 0, 1023, 0, 255);

//allows wheels to run opposite direction
  digitalWrite(dirpin_backleft, HIGH); 
  digitalWrite(dirpin_backright, HIGH); 

  analogWrite(pwmpin_backleft, pwmpin_val);
  analogWrite(pwmpin_backright, pwmpin_val);

  // ADD for front wheels

  //allows wheels to run opposite direction
  digitalWrite(dirpin_frontleft, HIGH); 
  digitalWrite(dirpin_frontright, HIGH); 

  analogWrite(pwmpin_frontleft, pwmpin_val);
  analogWrite(pwmpin_frontright, pwmpin_val);

  Serial.print("Motor speed pwm command: ");
  Serial.print(pwmpin_val);
  Serial.println();
}

