// ME350 Encoder Test
// Updated 10/13/2022

//------------------------------ Computation of position and velocity: ------------------------------//
// VARIABLES:
volatile long motorPosition = 0;       // [encoder counts] Current motor position (Declared 'volatile', since it is updated in a function called by interrupts)
volatile int encoderStatus = 0;       // [binary] Past and Current A&B values of the encoder  (Declared 'volatile', since it is updated in a function called by interrupts)
// The rightmost two bits of encoderStatus will store the encoder values from the current iteration (A and B).
// The two bits to the left of those will store the encoder values from the previous iteration (A_old and B_old).

//------------------------------ Encoder Count Positions:  ------------------------------//
// CONSTANTS:
// Target positions:
const int TARGET_1_POSITION    = 0;                    // [encoder counts] Motor position corresponding to target 1
const int TARGET_2_POSITION    = 119;                  // [encoder counts] Motor position corresponding to target 2
const int TARGET_3_POSITION    = 245;                  // [encoder counts] Motor position corresponding to target 3
const int TARGET_4_POSITION    = 565;                  // [encoder counts] Motor position corresponding to target 4

// Wait position (the position where your linkage waits until a target is detected)
const int WAIT_POSITION        = 280;                  // [encoder counts] Motor position corresponding to wait position 

//------------------------------ Pin assignment: ------------------------------//
// CONSTANTS:
const int PIN_NR_ENCODER_A        = 2;  // Never change these, since the interrupts are attached to pins 2 and 3
const int PIN_NR_ENCODER_B        = 3;  // Never change these, since the interrupts are attached to pins 2 and 3
const int PIN_NRL_LIMIT_SWITCH    = 8; // Connected to limit switch (mechanism calibration)



//////////////////////////////////////////////////////////////////////////////////////////
// The setup() function is called when a sketch starts. Use it to initialize variables, //
// pin modes, start using libraries, etc. The setup function will only run once, after  //
// each powerup or reset of the Arduino board:                                          //
//////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  // Declare which digital pins are inputs and which are outputs:
  pinMode(PIN_NR_ENCODER_A,        INPUT_PULLUP);
  pinMode(PIN_NR_ENCODER_B,        INPUT_PULLUP);
  pinMode(PIN_NRL_LIMIT_SWITCH,    INPUT);
  
  // Turn on the pullup resistors on the encoder channels
  digitalWrite(PIN_NR_ENCODER_A, HIGH);  
  digitalWrite(PIN_NR_ENCODER_B, HIGH);

  // Activate interrupt for encoder pins.
  // If either of the two pins changes, the function 'updateMotorPosition' is called:
  attachInterrupt(0, updateMotorPosition, CHANGE);  // Interrupt 0 is always attached to digital pin 2
  attachInterrupt(1, updateMotorPosition, CHANGE);  // Interrupt 1 is always attached to digital pin 3

  // Begin serial communication for monitoring.
  Serial.begin(115200);
  Serial.println("Start Executing Program.");
}
//------------------------------ End of function setup() ------------------------------//


////////////////////////////////////////////////////////////////////////////////////////////////
// After going through the setup() function, which initializes and sets the initial values,   //
// the loop() function does precisely what its name suggests, and loops consecutively,        //
// allowing your program to sense and respond. Use it to actively control the Arduino board.  //
//////////////////////////////////////////////////////////////////////////////////////////////// 
void loop() {
   if (digitalRead(PIN_NRL_LIMIT_SWITCH)==HIGH) { 
        // We reached the endstop.  Update the motor position to the limit:
        motorPosition = 0;  
  } 

  Serial.print("      Position [encoder counts]: ");
  Serial.println(motorPosition);
}
//------------------------------ End of function loop() ------------------------------//


//////////////////////////////////////////////////////////////////////
// This is a function to update the encoder count in the Arduino.   //
// It is called via an interrupt whenever the value on encoder      //
// channel A or B changes.                                          //
//////////////////////////////////////////////////////////////////////
void updateMotorPosition() {
  // Bitwise shift left by one bit, to make room for a bit of new data:
  encoderStatus <<= 1;   
  // Use a compound bitwise OR operator (|=) to read the A channel of the encoder (pin 2)
  // and put that value into the rightmost bit of encoderStatus:
  encoderStatus |= digitalRead(2);   
  // Bitwise shift left by one bit, to make room for a bit of new data:
  encoderStatus <<= 1;
  // Use a compound bitwise OR operator  (|=) to read the B channel of the encoder (pin 3)
  // and put that value into the rightmost bit of encoderStatus:
  encoderStatus |= digitalRead(3);
  // encoderStatus is truncated to only contain the rightmost 4 bits by  using a 
  // bitwise AND operator on mstatus and 15(=1111):
  encoderStatus &= 15;
  if (encoderStatus==2 || encoderStatus==4 || encoderStatus==11 || encoderStatus==13) {
    // the encoder status matches a bit pattern that requires counting up by one
    motorPosition++;         // increase the encoder count by one
  } 
  else if (encoderStatus == 1 || encoderStatus == 7 || encoderStatus == 8 || encoderStatus == 14) {
    // the encoder status does not match a bit pattern that requires counting up by one.  
    // Since this function is only called if something has changed, we have to count downwards
    motorPosition--;         // decrease the encoder count by one
  }
}
//------------------------------ End of function updateMotorPosition() ------------------------------//
