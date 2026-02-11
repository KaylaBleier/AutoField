#include <Servo.h>

Servo myServo;  

int servoPin = 9;   
int pos = 0;      

void setup() {
  myServo.attach(servoPin); 
}

void loop() {
  // Sweep from 0 to 40 degrees
  for (pos = -20; pos <= 40; pos += 1) {
    myServo.write(pos);
    delay(15);  
  }

  delay(3000);  // pause for 3 seconds 

  // Sweep from 40 to 0 degrees
  for (pos = 40; pos >= -20; pos -= 1) {
    myServo.write(pos);
    delay(15);
  }

  delay(3000);  // pause for 3 seconds
}


