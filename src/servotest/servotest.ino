#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position



void setup() {
  Serial.begin(9600);
  myservo.attach(3);  // attaches the servo on pin 9 to the servo object
  pinMode(5, INPUT);
}

int angle = 0;

void loop() {
  for (pos = angle; pos <= 180-angle; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);    
    int val = analogRead(14);
    Serial.print("Potentiometer:");
    Serial.print(val);
    Serial.print(",");
    Serial.print("Position:");
    Serial.println(pos);
    delay(100);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180-angle; pos >= angle; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);    
    int val = analogRead(0);
    Serial.print("Potentiometer:");
    Serial.print(val);
    Serial.print(",");
    Serial.print("Position:");
    Serial.println(pos);
    delay(100);                       // waits 15ms for the servo to reach the position
  }
}