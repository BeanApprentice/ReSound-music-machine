#include <Servo.h>

// the servos that will strike the bottles
const int sBottle1Pin = 4;
const int sBottle2Pin = 5;
const int sBottle3Pin = 6;
const int sBottle4Pin = 7;

// the servos that will plunk the rubber bands
const int sBand1Pin = 8;
const int sBand2Pin = 9;
const int sBand3Pin = 10;
const int sBand4Pin = 11;

Servo servoBottle1;
Servo servoBottle2;
Servo servoBottle3;
Servo servoBottle4;

Servo servoBand1;
Servo servoBand2;
Servo servoBand3;
Servo servoBand4;

void testServo(int dur) {
  servoBottle1.write(0);
  delay(dur/4);
  servoBottle1.write(90);
  delay(dur/4);
  servoBottle1.write(180);
  delay(dur/4);
  servoBottle1.write(90);
  delay(dur/4);
}

void setup() {
  servoBottle1.attach(sBottle1Pin);
  //testServo(4000);
}

void loop() {
  
}
