const int potpin = A0; //analog pin used to connect the center pin of potentiometer

// connections to arduino
int pwmpin_backleft = 3;
int pwmpin_backright = 5;
int dirpin_backleft = 2;
int dirpin_backright = 4;

// ADD for 4 motors, set variables for each wheel for pwp and direction

int pwmpin_val;


void setup() {
  // put your setup code here, to run once:

 // back wheel pwm and direction outputs
  pinMode(pwmpin_backleft, OUTPUT);
  pinMode(pwmpin_backright, OUTPUT);

  pinMode(dirpin_backleft, OUTPUT);
  pinMode(dirpin_backright, OUTPUT);

 // ADD front wheel pwm and direction outputs


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

  Serial.print("Motor speed pwm command: ");
  Serial.print(pwmpin_val);
  Serial.println();
}
