#include <Servo.h>

Servo myServo;  

int servoPin = 12;   
int pos = 0;      

int lower_bound = 0; 
int upper_bound = 29;

void setup() {
  myServo.attach(servoPin); 
}

void loop() {
  // Sweep up
  for (pos = lower_bound; pos <= upper_bound; pos++) {
    myServo.write(pos);
    delay(50);  
  }

  delay(1000);

  // Sweep down
  for (pos = upper_bound; pos >= lower_bound; pos--) {
    myServo.write(pos);
    delay(50);
  }

  delay(1000);
}
