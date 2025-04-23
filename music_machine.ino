// play music on a series of bottles & elastics using drumsticks swung by servos. The user selects which track they want to play with an IR remote. 
// uses a system that allows the program to play music based on pre-defined tracks, or based on live user inputs from a remote control / playing device. This is because the code that converts given notes into bottle strikes is independant from the code that processes the track arrays. 
// Runs on a NANO ATmega328 with old bootloader.
// Written by Noah Broccolini.

#include <Servo.h>

// rather than storing frequencies, this array stores the positions of bottles that are presumed to play certain frequencies

const int track1Bottles[] = {0, 1,   2,   3,   4,    5,    6,    7,    6,    5,    4,    3,    2,    1,    0}; // test all the bottles
const int track1Times[] = {  0, 250, 500, 750, 1000, 1250, 1500, 1750, 2000, 2250, 2500, 2750, 3000, 3250, 3500};

const int track2Bottles[] = {0, 0, 0, 1, 2, 1, 0, 2, 1, 1, 0, // o claire de la lune 
                             0, 0, 0, 1, 2, 1, 0, 2, 1, 1, 0};
const int track2Times[] = {0,    333,  667,  1000, 1333, 2000, 2667, 3000, 3333, 3667, 4000,
                           5333, 5667, 6000, 6333, 6667, 7333, 8000, 8333, 8667, 9000, 9333};

const int track3Bottles[] = {2, 1, 0, 1, 2, 2, 2, 1, 1, 1, 2, 3, 3, // mary had a little lamb
                             2, 1, 0, 1, 2, 2, 2, 2, 1, 1, 2, 1, 0}; 
const int track3Times[] = {0,    250,  500,  750,  1000, 1250, 1500, 2000, 2250, 2500, 3000, 3250, 3500,
                           4000, 4250, 4500, 4750, 5000, 5250, 5500, 5750, 6000, 6250, 6500, 6750, 7000};

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;
Servo servo7;
Servo servo8;

const int restAngle = 90;
const int strikeAngleDelta = 8; // how much the servo has to turn from rest in order to strike the bottle
const int strikeDuration = 40; // amount of milliseconds to set a servo to the strike position before beginning to retract it.
const int servoOffsets[8] = {-2, 3, 10, 0, 0, 0, 0, 0}; // calibrated offsets to make sure each servo is aligned in spite of physical tolerances in the servo shafts.

unsigned long now = 0; // for keeping track of the current time since the arduino booted, for all the servo strike timings
unsigned long trackBeginTime = 0; // for keeping track of when the current track began playing. The individual notes are marked for the elapsed time at which they are played.
unsigned int nextNote = 0; // for keeping track of which is the next index on the list to be played (once the time to play that note arrives). 
bool playing = 0; // if this is true, then the program will play a track from the given arrays until the nextNote variable exceeds the length of the notes array of the selected track.
float playSpeed = 1; // change how fast the tracks play by multiplying or dividing the timestamps of each note by a certain factor.

bool striking[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // store whether any of the given servos are in the process of striking. The strike function sets a value within this array to 1, and a seperate function returns any 1s back to 0s after a certain time.
unsigned long strikeTimes[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // remember at what time a given servo began its strike, so that the retraction can be properly timed



void strikeBottle(int botServ) { // sets the given servo's striking status to 1 and marks the beginning of the strike in time
  strikeTimes[botServ] = now;
  striking[botServ] = 1;
  digitalWrite(LED_BUILTIN, 1);
}

void resetFinishedServos() { // reset the angle of any servos whose striking state is at 1, but whose elapsed time since the servo struck has exceeded the strike duration
  for (int serv = 0; serv <= 7; serv++) {
    if ( striking[serv] && ((now - strikeTimes[serv]) >= strikeDuration) ) {
    striking[serv] = 0;
    digitalWrite(LED_BUILTIN, 0);
    }
  }
}

void updateServoStates() { // based on which positions in the striking array are set to 1, change the physical state of servos to the striking angle, and the others to the retracted angle.
  resetFinishedServos();
  
  if (striking[0]) {servo1.write(restAngle+strikeAngleDelta+servoOffsets[0]);} // the servo control is object-oriented, so I have to individually address each servo by name.
  else {servo1.write(restAngle+servoOffsets[0]);}

  if (striking[1]) {servo2.write(restAngle-strikeAngleDelta-servoOffsets[1]);} // every other servo has its added angle values flipped, because it is facing the opposite direction
  else {servo2.write(restAngle-servoOffsets[1]);}

  if (striking[2]) {servo3.write(restAngle+strikeAngleDelta+servoOffsets[2]);}
  else {servo3.write(restAngle+servoOffsets[2]);}

  if (striking[3]) {servo4.write(restAngle-strikeAngleDelta-servoOffsets[3]);}
  else {servo4.write(restAngle-servoOffsets[3]);}

  if (striking[4]) {servo5.write(restAngle+strikeAngleDelta+servoOffsets[4]);}
  else {servo5.write(restAngle+servoOffsets[4]);}

  if (striking[5]) {servo6.write(restAngle-strikeAngleDelta-servoOffsets[5]);}
  else {servo6.write(restAngle-servoOffsets[5]);}

  if (striking[6]) {servo7.write(restAngle+strikeAngleDelta+servoOffsets[6]);}
  else {servo7.write(restAngle+servoOffsets[6]);}

  if (striking[7]) {servo8.write(restAngle-strikeAngleDelta-servoOffsets[7]);}
  else {servo8.write(restAngle-servoOffsets[7]);}
}



void setup() {
  servo1.attach(2);
  servo2.attach(3);
  servo3.attach(4);
  servo4.attach(5);
  servo5.attach(6);
  servo6.attach(7);
  servo7.attach(8);
  servo8.attach(9);
  pinMode(A0, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  servo1.write(restAngle);
  servo2.write(restAngle);
  servo3.write(restAngle);
  servo4.write(restAngle);
  servo5.write(restAngle);
  servo6.write(restAngle);
  servo7.write(restAngle);
  servo8.write(restAngle);

  delay(500);

  /*trackBeginTime = now;
  nextNote = 0;
  playing = 1;*/
}

void loop() { // this program is NON-BLOCKING. Adding blocking code may break cause everything to break.
  now = millis(); // update this variable before every loop


  
  if (!playing && !digitalRead(A0)) { // start playing a track when the test button is pressed
    trackBeginTime = now;
    nextNote = 0;
    playing = 1;
  }
  
  if (playing && ( (now-trackBeginTime) >= track3Times[nextNote]/playSpeed) ) { // as the elapsed time progresses, the next note in the song will be periodically reached, at which point the command will be sent for the appropriate bottle to be struck, and the checkpoint will move forward by one note.
    strikeBottle(track3Bottles[nextNote]);
    nextNote++;
  }

  if ( nextNote > (sizeof(track3Times) / sizeof(track3Times[0])) ) { // figure out what the last note of a track is based on the amount of byte the track takes up, divided by the amount of bytes used by each note.
    playing = 0; // stop playing the track once the last note has been played.
  }



  updateServoStates(); // this function needs to be called after every loop, or the servos will not move properly.
}
