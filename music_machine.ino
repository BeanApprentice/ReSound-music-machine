// play music on a series of bottles & elastics using drumsticks swung by servos. The user selects which track they want to play with an IR remote. 
// uses a system that allows the program to play music based on pre-defined tracks, or based on live user inputs from a remote control / playing device. This is because the code that converts given notes into bottle strikes is independant from the code that processes the track arrays. 
// !!! Runs on a NANO ATmega328 with old bootloader !!!
// Written by Noah Broccolini.

#define DECODE_NEC // Includes Apple and Onkyo. To enable all protocols , just comment/disable this line.

enum remoteCommands { // clarify all the possible IR button hex codes, so that human-readable words can be used to compare to the IrReceiver.decodedIRData.command value
  POWER = 0x45, // used to select alarm mode
  MENU = 0x47, // used to exit the various modes, ie play mode, test mode, write mode
  TEST = 0x44,
  PLUS = 0x40,
  LOOP = 0x43,
  TRACKBACK = 0x7,
  PLAY = 0x15,
  TRACKFORWARD = 0x9,
  ZERO = 0x16,
  MINUS = 0x19,
  CLOCK = 0xD, // used to activate timer mode (alternate idea: compose mode)
  ONE = 0xC,
  TWO = 0x18,
  THREE = 0x5E,
  FOUR = 0x8,
  FIVE = 0x1C,
  SIX = 0x5A,
  SEVEN = 0x42,
  EIGHT = 0x52,
  NINE = 0x4A
};

#include <IRremote.hpp>
#include <Servo.h>

// rather than storing frequencies, this array stores the positions of bottles that are presumed to play certain frequencies
// a position of -1 indicates a rest. Since rests are already made in between notes, a -1 is added only at the end of a song in order to create a delay between two songs if another one is to be played.

const int track00Bottles[16] = {2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2}; // test one bottle repeatedly for tuning
const int track00Times[16] = {  0,100,200,300,400,500,600,700,800,900,1000,1100,1200,1300,1400,1500};

const int alarmBottles[] = {0,1,2,3,4,5,6,7, 0,1,2,3,4,5,6,7, 0,1,2,3,4,5,6,7, 0,1,2,3,4,5,6,7, 0,1,2,3,4,5,6,7, 0,1,2,3,4,5,6,7, 0,1,2,3,4,5,6,7, 0,1,2,3,4,5,6,7, 0,1,2,3,4,5,6,7, 0,1,2,3,4,5,6,7, 0,1,2,3,4,5,6,7, 0,1,2,3,4,5,6,7, 0,1,2,3,4,5,6,7, 0,1,2,3,4,5,6,7, 0,1,2,3,4,5,6,7, 0,1,2,3,4,5,6,7, -1};
const int alarmTimes[] = {  0,0,0,0,0,0,0,0, 100,100,100,100,100,100,100,100, 200,200,200,200,200,200,200,200, 300,300,300,300,300,300,300,300, 400,400,400,400,400,400,400,400, 500,500,500,500,500,500,500,500, 600,600,600,600,600,600,600,600, 700,700,700,700,700,700,700,700, 800,800,800,800,800,800,800,800, 900,900,900,900,900,900,900,900, 1000,1000,1000,1000,1000,1000,1000,1000, 1100,1100,1100,1100,1100,1100,1100,1100, 1200,1200,1200,1200,1200,1200,1200,1200, 1300,1300,1300,1300,1300,1300,1300,1300, 1400,1400,1400,1400,1400,1400,1400,1400, 1500,1500,1500,1500,1500,1500,1500,1500, 2400};

const int track0Bottles[] = {0, 1,   2,   3,   4,    5,    6,    7,    6,    5,    4,    3,    2,    1,    0}; // test all the bottles
const int track0Times[] = {  0, 250, 500, 750, 1000, 1250, 1500, 1750, 2000, 2250, 2500, 2750, 3000, 3250, 3500};

const int track1Bottles[] = {0, 0, 0, 1, 2, 1, 0, 2, 1, 1, 0, // o claire de la lune -  3 bottles
                             0, 0, 0, 1, 2, 1, 0, 2, 1, 1, 0, -1};
const int track1Times[] = {0,    333,  667,  1000, 1333, 2000, 2667, 3000, 3333, 3667, 4000,
                           5333, 5667, 6000, 6333, 6667, 7333, 8000, 8333, 8667, 9000, 9333, 10666};

const int track2Bottles[] = {2, 1, 0, 1, 2, 2, 2, 1, 1, 1, 2, 3, 3, // mary had a little lamb - 4 bottles
                             2, 1, 0, 1, 2, 2, 2, 2, 1, 1, 2, 1, 0, -1}; 
const int track2Times[] = {0,    250,  500,  750,  1000, 1250, 1500, 2000, 2250, 2500, 3000, 3250, 3500,
                           4000, 4250, 4500, 4750, 5000, 5250, 5500, 5750, 6000, 6250, 6500, 6750, 7000, 8000};

const int track3Bottles[] = {}; // reveille - 5 bottles 
const int track3Times[] = {};

const int track4Bottles[] = {0, 7, 0, 7, 0, 7, 1, 0, 2,  3, 2, 3, 2, 1, 0, 7, 7, 0, 7, -1}; // unknown - 8 bottles
const int track4Times[] = {0, 250, 500, 750, 1000, 1500, 2000, 2500, 3000,  4000, 4250, 4500, 4750, 5000, 5250, 5500, 5750, 6000, 7000, 8000};

Servo servo1; // change these to 0-7 for a more streamlined nomenclature
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
const int servoOffsets[8] = {7, 7, 1, 3, -6, 1, 1, -1}; // calibrated offsets to make sure each servo is aligned in spite of physical tolerances in the servo shafts.

unsigned long now = 0; // for keeping track of the current time since the arduino booted, for all the servo strike timings
unsigned long trackBeginTime = 0; // for keeping track of when the current track began playing. The individual notes are marked for the elapsed time at which they are played.
unsigned int nextNote = 0; // for keeping track of which is the next index on the list to be played (once the time to play that note arrives). 

bool playing = 0; // if this is true, then the program will play a track from the given arrays until the nextNote variable exceeds the length of the notes array of the selected track.
bool looped = 0; // if true, then the track will loop indefinitely until this is false again.
bool testMode = 0; // manually swing the servos using digits 1-8 on the numberpad
bool timerMode = 0; // play an alarm sound after a user-selected delay
bool timerRunning = 0;

float playSpeed = 1.0; // change how fast the tracks play by multiplying or dividing the timestamps of each note by a certain factor.
int selectedTrack = 0;
float oldSpeed = 1.0; // the alarm needs to change the user's selected track and playback speed, so remember their previous settings so they can be reset after the alarm ends
int oldTrack = 0;

unsigned int timerValue = 0; // the delay in minutes before the timer alarm rings
unsigned long timerBeginTime = 0;


bool buttonRisingEdge = 0; // if the user holds down a button on the remote, this value allows the program to distinguish between the first signal of that button press, and any subsequent signals from that same button press

bool striking[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // store whether any of the given servos are in the process of striking. The strike function sets a value within this array to 1, and a seperate function returns any 1s back to 0s after a certain time.
unsigned long strikeTimes[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // remember at what time a given servo began its strike, so that the retraction can be properly timed



void strikeBottle(int botServ) { // sets the given servo's striking status to 1 and marks the beginning of the strike in time
  if (botServ >= 0 && botServ <= 7) { // make sure that the servo selection received is a valid position in the striking lists, otherwise the arduino may overwrite an undesired memory location.
    strikeTimes[botServ] = now;
    striking[botServ] = 1;
    digitalWrite(LED_BUILTIN, 1);
  }
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
  
  if (striking[0]) {servo1.write(restAngle-strikeAngleDelta-servoOffsets[0]);} // the servo control is object-oriented, so I have to individually address each servo by name.
  else {servo1.write(restAngle-servoOffsets[0]);}

  if (striking[1]) {servo2.write(restAngle+strikeAngleDelta+servoOffsets[1]);} // every other servo has its added angle values flipped, because it is facing the opposite direction
  else {servo2.write(restAngle+servoOffsets[1]);}

  if (striking[2]) {servo3.write(restAngle-strikeAngleDelta-servoOffsets[2]);}
  else {servo3.write(restAngle-servoOffsets[2]);}

  if (striking[3]) {servo4.write(restAngle+strikeAngleDelta+servoOffsets[3]);}
  else {servo4.write(restAngle+servoOffsets[3]);}

  if (striking[4]) {servo5.write(restAngle-strikeAngleDelta-servoOffsets[4]);}
  else {servo5.write(restAngle-servoOffsets[4]);}

  if (striking[5]) {servo6.write(restAngle+strikeAngleDelta+servoOffsets[5]);}
  else {servo6.write(restAngle+servoOffsets[5]);}

  if (striking[6]) {servo7.write(restAngle-strikeAngleDelta-servoOffsets[6]);}
  else {servo7.write(restAngle-servoOffsets[6]);}

  if (striking[7]) {servo8.write(restAngle+strikeAngleDelta+servoOffsets[7]);}
  else {servo8.write(restAngle+servoOffsets[7]);}
}

int getNumberPad(int IrCmd) { // converts all the number pad hex codes into actual numbers corresponding to each button
  
  switch (IrCmd) {
    case ZERO:
      return 0;
    case ONE:
      return 1;
    case TWO:
      return 2;
    case THREE:
      return 3;
    case FOUR:
      return 4;
    case FIVE:
      return 5;
    case SIX:
      return 6;
    case SEVEN:
      return 7;
    case EIGHT:
      return 8;
    case NINE:
      return 9;
    default:
      return -1;
  }
  
}

unsigned long getNoteTime(int track, int index) { // returns the time at which the note at a given index within the selected track must be played
  
  switch (track) {
    case 0:
      return track0Times[index];
    case 1:
      return track1Times[index];
    case 2:
      return track2Times[index];
    case 3:
      return track3Times[index];
    case 4:
      return track4Times[index];
    case 9: // alarm sound cannot be selected by the number pad
      return alarmTimes[index];
    default: // default output indicates that the selected track does not exist
      return 0;
  }
  
}

int getNotePosition(int track, int index) { // returns the position of the bottle to strike for a note at a given index within the selected track

  switch (track) {
    case 0:
      return track0Bottles[index];
    case 1:
      return track1Bottles[index];
    case 2:
      return track2Bottles[index];
    case 3:
      return track3Bottles[index];
    case 4:
      return track4Bottles[index];
    case 9:
      return alarmBottles[index];
    default:
      return -1;
  }
  
}

int getTrackLength(int track) { // figure out what the last note of a track is based on the amount of bytes the track takes up, divided by the amount of bytes used by each note.
  
  switch (track) {
    case 0:
      return (sizeof(track0Times) / sizeof(track0Times[0]));
    case 1:
      return (sizeof(track1Times) / sizeof(track1Times[0]));
    case 2:
      return (sizeof(track2Times) / sizeof(track2Times[0]));
    case 3:
      return (sizeof(track3Times) / sizeof(track3Times[0]));
    case 4:
      return (sizeof(track4Times) / sizeof(track4Times[0]));
    case 9:
      return (sizeof(alarmTimes) / sizeof(alarmTimes[0]));
    default:
      return -1;
  }
  
}



void setup() {
  servo1.attach(3);
  servo2.attach(4);
  servo3.attach(5);
  servo4.attach(8);
  servo5.attach(9); // may have to move these servos to A0-A3 if the SPI pins are needed for radio communication
  servo6.attach(10);
  servo7.attach(11);
  servo8.attach(2);
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

  delay(250);

  IrReceiver.begin(A1, false); // setup the IR receiver on pin A1 (it only needs a digital input if available). False means no LED feedback on the arduino, since ther's already a feedback LED on the IR chip.

  delay(250);

  //Serial.begin(115200);
  //Serial.println("ReSound booted...");

  /*trackBeginTime = now;
  nextNote = 0;
  playing = 1;*/
}

void loop() { // this program is NON-BLOCKING. Adding blocking code may cause everything to break.
  now = millis(); // update this variable before every loop
  int IRcommand; // the value will be empty every loop by default, unless a known command is decoded

  if (IrReceiver.decode()) { // && IrReceiver.decodedIRData.protocol != UNKNOWN
    IrReceiver.resume(); // Enable receiving of the next value
    IRcommand = IrReceiver.decodedIRData.command;
    buttonRisingEdge = !IrReceiver.decodedIRData.flags;

    /*Serial.println(IRcommand);
    Serial.println(signalRepeated);
    Serial.println("");*/

    /*if (buttonRisingEdge) {
      Serial.print("Button just pressed");Serial.println("");
    }*/
  }
  else {
    buttonRisingEdge = 0; // if no signal is decoded, it means either that no button is being pressed, or the remote's signal is not reaching the receiver. Either way, reset the rising edge detector, because if it was previously known to be a rising edge, it MUST no longer be a rising edge after one repeat of the loop().
  }



  bool startCondition = (looped || !digitalRead(A0) || ( IRcommand == PLAY && buttonRisingEdge )); // start conditions for regular music playing: either the song is looping, the onboard start button is pressed, or the remote control play button is clicked
  bool alarmCondition = timerMode && timerRunning && !playing && (now-timerBeginTime >= timerValue*60000); // A condition that triggers an alternate version of the playing code so that the machine plays an alarm instead of its normal music. Multiply the timer value in minutes by 60k to get milliseconds.

  if (IRcommand == MENU) { // exit any current mode and reset some parameters if the menu button is pressed. The user cannot switch directly from one mode to another, and must instead go through the menu first, so any parameters from a mode that must get reset upon exiting will be reset here.
    playing = 0;
    looped = 0;
    //playSpeed = 1.0;

    testMode = 0;

    // the alarm can be stopped at any time by pressing the menu button, or reset once it begins ringing by pressing the play button again. EDIT: ok, maybe just the menu button
    if (timerMode) {
      timerMode = 0; 
      timerValue = 0;
      timerRunning = 0; 

      selectedTrack = oldTrack;
      playSpeed = oldSpeed;
    }
    
  }
  
  if  (!playing && !testMode && !timerMode) { // when the machine is in the menu, allow the user to select which track they want to be played
    if (getNumberPad(IRcommand) != -1) { // -1 is the default output of this function, if it reads no / an invalid input from the remote
      selectedTrack = getNumberPad(IRcommand);
    }
  }

  if (!playing && IRcommand == PLUS && buttonRisingEdge) { // can only control the playback speed when not playing, otherwise the track will skip forward or backward due to how the track reading is programmed...
    playSpeed += 0.25;
    if (playSpeed >= 2.0) {playSpeed = 2.0;}
    //Serial.print("Increasing playback speed to x"); Serial.print(playSpeed); Serial.println("");
  }
  if (!playing && IRcommand == MINUS && buttonRisingEdge) {
    playSpeed -= 0.25;
    if (playSpeed <= 0.25) {playSpeed = 0.25;}
    //Serial.print("Decreasing playback speed to x"); Serial.print(playSpeed); Serial.println("");
  }

  
  if (( !playing && !testMode && !timerMode && startCondition ) || alarmCondition) { // start playing a track if a mode is not currently running, and one of the start conditions is met... or if an alarm must be sounded
    trackBeginTime = now;
    nextNote = 0;
    playing = 1;
    //Serial.print("Starting song at ");Serial.print(now);Serial.println("ms");

    if (alarmCondition) { // extra setup for the alarm
      selectedTrack = 9; // track 10 is the alarm track, not selectable by the user through the numberpad
      playSpeed = 1.0;
      looped = 1;
      //Serial.println("ALARM!");
    }
  }
  
  if ( playing && ( (now-trackBeginTime) >= getNoteTime(selectedTrack, nextNote)/playSpeed) ) { // as the elapsed time progresses, the next note in the song will be periodically reached, at which point the command will be sent for the appropriate bottle to be struck, and the checkpoint will move forward by one note.
    strikeBottle(getNotePosition(selectedTrack, nextNote));
    //Serial.print("Playing note ");Serial.print(nextNote);Serial.print(" at ");Serial.print(now);Serial.print("ms: bottle ");Serial.println(track00Bottles[nextNote]);
    nextNote++;
  }

  if ( playing && nextNote >= getTrackLength(selectedTrack) ) {
    playing = 0; // stop playing the track once the last note has been played. It may restart again immediately if in looping mode.
    //Serial.print("Ending song at ");Serial.print(now);Serial.println("ms");
  }

  if (playing && IRcommand == LOOP /*&& !signalRepeated*/) { // looping mode can be selected when in playing mode
    looped = 1;
  }

  if (!playing && !testMode && !timerMode && IRcommand == TEST) {
    testMode = 1;
  }

  if (testMode && getNumberPad(IRcommand) >= 1 && getNumberPad(IRcommand) <= 8 && buttonRisingEdge) { // strike the bottle that the user selects
    strikeBottle(getNumberPad(IRcommand)-1); // minus 1 because the bottles are indexed 0-7
  }

  if (!playing && !testMode && !timerMode && IRcommand == CLOCK) {
    timerMode = 1;
  }

  if (timerMode && getNumberPad(IRcommand) != -1 && buttonRisingEdge) {
    if (timerValue*10 + getNumberPad(IRcommand) > 1440) { // stop the timer value from exceeding 1 day
      
    }
    else { // If the timer value is empty, it is replaced with the digit pressed. If it already has a number in it, it is multiplied by 10 before adding the next number into the units position.
      timerValue = timerValue*10;
      timerValue += getNumberPad(IRcommand);

      //Serial.print("Timer set to ");Serial.print(timerValue);Serial.print("min");Serial.println("");
    }
  }

  if (timerMode && IRcommand == PLAY && !timerRunning) { // begin the timer and let the user know by striking a bottle 
    timerBeginTime = now;
    timerRunning = 1;
    oldTrack = selectedTrack;
    oldSpeed = playSpeed;
    strikeBottle(7);
  }

  /*if (timerMode && timerRunning) { // now-timerBeginTime >= timerValue*60000
    Serial.print("Time elapsed: ");Serial.print(now-timerBeginTime);Serial.println("ms");
    Serial.print("Target time: ");Serial.print(timerValue*60000);Serial.println("ms");
    Serial.println("");
  }*/



  updateServoStates(); // this function needs to be called after every loop, or the servos will not move properly.
}
