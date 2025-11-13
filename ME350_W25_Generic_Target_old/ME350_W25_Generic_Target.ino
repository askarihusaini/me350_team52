/* ME350 Encoder Test
   Updated 10/13/2022 */


//------------------------------ Computation of position and velocity: ------------------------------//
// CONSTANTS: 
// Settings for velocity computation:
const int MIN_VEL_COMP_COUNT = 2;     // [encoder counts] Minimal change in motor position that must happen between two velocity measurements
const long MIN_VEL_COMP_TIME = 10000; // [microseconds] Minimal time that must pass between two velocity measurements
// VARIABLES:
volatile long motorPosition = 0;       // [encoder counts] Current motor position (Declared 'volatile', since it is updated in a function called by interrupts)
volatile int encoderStatus = 0;       // [binary] Past and Current A&B values of the encoder  (Declared 'volatile', since it is updated in a function called by interrupts)
// The rightmost two bits of encoderStatus will store the encoder values from the current iteration (A and B).
// The two bits to the left of those will store the encoder values from the previous iteration (A_old and B_old).
float motorVelocity        = 0;       // [encoder counts / seconds] Current motor velocity 
int previousMotorPosition  = 0;       // [encoder counts] Motor position the last time a velocity was computed 
long previousVelCompTime   = 0;       // [microseconds] System clock value the last time a velocity was computed 

//------------------------------ Encoder Count Positions:  ------------------------------//
// CONSTANTS:
// Target positions:
const int TARGET_1_POSITION    = 0;                    // [encoder counts] Motor position corresponding to target 1
const int TARGET_2_POSITION    = 100;                  // [encoder counts] Motor position corresponding to target 2
const int TARGET_3_POSITION    = 200;                  // [encoder counts] Motor position corresponding to target 3
const int TARGET_4_POSITION    = 400;                  // [encoder counts] Motor position corresponding to target 4
const int WAIT_POSITION        = 250;                  // [encoder counts] Motor position corresponding to wait position 
const int LOWER_BOUND             = 0;                 // [encoder counts] Position of the left end stop
const int UPPER_BOUND             = 500;               // [encoder counts] Position of the right end stop
const int TARGET_BAND             = 5;                 // [encoder counts] "Close enough" range when moving towards a target.

//------------------------------ Motor Constants:  ------------------------------//
// CONSTANTS:
const float SUPPLY_VOLTAGE        = 9.0;  // [Volt] Supply voltage at the HBridge
const float FRICTION_COMP_VOLTAGE = 4.3;   // [Volt] Voltage needed to overcome friction
// VARIABLES:
float desiredVoltage = 0;                 // [Volt] Desired motor voltage
int   motorCommand   = 0;                 // [0-255] PWM signal sent to the motor
unsigned long executionDuration = 0;      // [microseconds] Time between this and the previous loop execution.  Variable used for integrals and derivatives
unsigned long lastExecutionTime = 0;      // [microseconds] System clock value at the moment the loop was started the last time

//------------------------------ Pin assignment: ------------------------------//
// CONSTANTS:
const int PIN_NR_ENCODER_A        = 2;  // Never change these, since the interrupts are attached to pins 2 and 3
const int PIN_NR_ENCODER_B        = 3;  // Never change these, since the interrupts are attached to pins 2 and 3
const int PIN_NR_ON_OFF_SWITCH    = 5;  // Connected to toggle switch (turns mechanism on and off)
const int PIN_NRL_LIMIT_SWITCH    = 8; // Connected to limit switch (mechanism calibration)
const int PIN_NR_PWM_OUTPUT       = 11; // Connected to H Bridge (controls motor speed)
const int PIN_NR_PWM_DIRECTION_1  = 12; // Connected to H Bridge (controls motor direction)
const int PIN_NR_PWM_DIRECTION_2  = 13; // Connected to H Bridge (controls motor direction)


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
  pinMode(PIN_NR_ON_OFF_SWITCH,    INPUT);
  pinMode(PIN_NR_PWM_OUTPUT,       OUTPUT);
  pinMode(PIN_NR_PWM_DIRECTION_1,  OUTPUT);
  pinMode(PIN_NR_PWM_DIRECTION_2,  OUTPUT);
  
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
  // Determine the duration it took to execute the last loop. This time is used 
  // for integration and for monitoring the loop time via the serial monitor.
  executionDuration = micros() - lastExecutionTime;
  lastExecutionTime = micros();

  // Speed Computation:
  if ((abs(motorPosition - previousMotorPosition) > MIN_VEL_COMP_COUNT) || (micros() - previousVelCompTime) > MIN_VEL_COMP_TIME){
    // If at least a minimum time interval has elapsed or
    // the motor has travelled through at least a minimum angle ... 
    // .. compute a new value for speed:
    // (speed = delta angle [encoder counts] divided by delta time [seconds])
    motorVelocity = (double)(motorPosition - previousMotorPosition) * 1000000 / 
                            (micros() - previousVelCompTime);
    // Remember this encoder count and time for the next iteration:
    previousMotorPosition = motorPosition;
    previousVelCompTime   = micros();
  }
 
 //******************************************************************************//
  // Position Controller
  if (digitalRead(PIN_NR_ON_OFF_SWITCH)==HIGH) {
    // If the toggle switch is on, run the controller:
 
    //** Friction Compensation: **//
    // Compensate for friction.  That is, if we now the direction of 
    // desired motion, add a command that helps with moving 
      desiredVoltage = FRICTION_COMP_VOLTAGE;
  }
  else {
    desiredVoltage = 0; 
    Serial.println("The toggle switch is off.  Motor Stopped.");
  }
  
  //** Send signal to motor **//
  // Convert from voltage to PWM cycle:
  motorCommand = int(abs(desiredVoltage * 255 / SUPPLY_VOLTAGE));
  // Clip values larger than 255
  if (motorCommand > 255) {
    motorCommand = 255;
  }
  // Send motor signals out
  analogWrite(PIN_NR_PWM_OUTPUT, motorCommand);
  // Determine rotation direction
  if (desiredVoltage >= 0) {
    // If voltage is positive ...
    // ... turn forward
    digitalWrite(PIN_NR_PWM_DIRECTION_1,LOW);  // rotate forward
    digitalWrite(PIN_NR_PWM_DIRECTION_2,HIGH); // rotate forward
  } 
  else {
    // ... otherwise turn backward:
    digitalWrite(PIN_NR_PWM_DIRECTION_1,HIGH); // rotate backward
    digitalWrite(PIN_NR_PWM_DIRECTION_2,LOW);  // rotate backward
  }
  // End of Position Controller
  //*********************************************************************//

  Serial.print("      Position [encoder counts]: ");
  Serial.print("  P: "); 
  Serial.println(motorPosition);
  Serial.print("      Desired Voltage:  ");
  Serial.println(desiredVoltage);
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
