// ME350 Target Dropping Sketch - Version 04.2 by Eli Parham
// updated 11-3-2022
//
//

//////////////////////////////////////////////
// DEFINE CONSTANTS AND GLOBAL VARIABLES:   //
//////////////////////////////////////////////

//** State Machine: **//
// CONSTANTS: 
// Definition of states in the state machine
const int CALIBRATE                = 1;
const int DETERMINE_ACTIVE_TARGETS = 2;
const int MOVE_TO_TARGET           = 3;

// VARIABLES:
// Global variable that keeps track of the state:
// Start the state machine in calibration state:
int state = CALIBRATE;

//** Proximity Sensors or Potentiometer: **//
// CONSTANTS: 
// Definition of proximity sensor thresholds for each target:
const int TARGET1_PROXIMITYSENSE1_MIN = 0;     // [proximity sensor counts] Min value prox sensor 1 reads when target 1 is active
const int TARGET2_PROXIMITYSENSE1_MIN = 0;     // [proximity sensor counts] Min value prox sensor 1 reads when target 2 is active
const int TARGET2_PROXIMITYSENSE1_MAX = 0;     // [proximity sensor counts] Max value prox sensor 1 reads when target 2 is active
const int TARGET3_PROXIMITYSENSE1_MIN = 0;     // [proximity sensor counts] Min value prox sensor 1 reads when target 3 is active
const int TARGET3_PROXIMITYSENSE1_MAX = 0;     // [proximity sensor counts] Max value prox sensor 1 reads when target 3 is active
const int TARGET3_PROXIMITYSENSE2_MIN = 0;     // [proximity sensor counts] Min value prox sensor 2 reads when target 3 is active
const int TARGET3_PROXIMITYSENSE2_MAX = 0;     // [proximity sensor counts] Max value prox sensor 2 reads when target 3 is active
const int TARGET4_PROXIMITYSENSE2_MIN = 0;     // [proximity sensor counts] Min value prox sensor 2 reads when target 4 is active
const int TARGET4_PROXIMITYSENSE2_MAX = 0;     // [proximity sensor counts] Max value prox sensor 2 reads when target 4 is active
const int TARGET5_PROXIMITYSENSE2_MIN = 0;     // [proximity sensor counts] Min value prox sensor 2 reads when target 5 is active
const int TARGETNONE_PROXIMITYSENSE1_MAX = 0;  // [proximity sensor counts] Max value prox sensor 1 reads when no target is active
const int TARGETNONE_PROXIMITYSENSE2_MAX = 0;  // [proximity sensor counts] Max value prox sensor 2 reads when no target is active
// Definition of target variables for array indexing
const int TARGET1  = 0;   // For indexing into activeTargets array, activeTargets[0] corresponds to Target 1
const int TARGET2  = 1;   // For indexing into activeTargets array, activeTargets[1] corresponds to Target 2
const int TARGET3  = 2;   // For indexing into activeTargets array, activeTargets[2] corresponds to Target 3
const int TARGET4  = 3;   // For indexing into activeTargets array, activeTargets[3] corresponds to Target 4
const int TARGET5  = 4;   // For indexing into activeTargets array, activeTargets[4] corresponds to Target 5
const int ACTIVE   = 1;   // In activeTargets array, a 1 means the corresponding target is active
const int INACTIVE = 0;   // In activeTargets array, a 0 means the corresponding target is inactive
const int UNSURE   = -1;  // In activeTargets array, a -1 means the corresponding target is not visible
// VARIABLES:
// Create variables that read and average proximity sensor or potentiometer readings:
int proximitySense1Values[20]; // Holds 20 proximity sensor readings for averaging
int proximitySense2Values[20]; // Holds 20 proximity sensor readings for averaging
float proximitySense1Avg = 0;  // Average of all values in proximitySense1Values
float proximitySense2Avg = 0;  // Average of all values in proximitySense2Values
int potValue = 0;              // Used to hold the reading of the potentiometer
// Variable to keep track of active targets:
int activeTargets[5] = {-1, -1, -1, -1, -1}; // Shows whether or not each of the five targets is active. -1 means the target is not visible.
                                             // 0 means the target is inactive. 1 means the target is active.
int activeTargetPosition = -1; // [encoder counts] Encoder count of the active target the linkage is moving toward

//** Computation of position and velocity: **//
// CONSTANTS: 
// Settings for velocity computation:
const int MIN_VEL_COMP_COUNT = 2;     // [encoder counts] Minimal change in motor position that must happen between two velocity measurements
const long MIN_VEL_COMP_TIME = 10000; // [microseconds] Minimal time that must pass between two velocity measurements
// VARIABLES:
volatile int motorPosition = 0; // [encoder counts] Current motor position (Declared 'volatile', since it is updated in a function called by interrupts)
volatile int encoderStatus = 0; // [binary] Past and Current A&B values of the encoder  (Declared 'volatile', since it is updated in a function called by interrupts)
// The rightmost two bits of encoderStatus will store the encoder values from the current iteration (A and B).
// The two bits to the left of those will store the encoder values from the previous iteration (A_old and B_old).
float motorVelocity        = 0; // [encoder counts / seconds] Current motor velocity 
int previousMotorPosition  = 0; // [encoder counts] Motor position the last time a velocity was computed 
long previousVelCompTime   = 0; // [microseconds] System clock value the last time a velocity was computed 

//** High-level behavior of the controller:  **//
// CONSTANTS:
// Target positions:
const int CALIBRATION_VOLTAGE  = -4; // [Volt] Motor voltage used during the calibration process
const int TARGET_1_POSITION    = 0; // [encoder counts] Motor position corresponding to first target
const int TARGET_2_POSITION    = 0; // [encoder counts] Motor position corresponding to second target
const int TARGET_3_POSITION    = 0; // [encoder counts] Motor position corresponding to third target
const int TARGET_4_POSITION    = 0; // [encoder counts] Motor position corresponding to fourth target
const int TARGET_5_POSITION    = 0; // [encoder counts] Motor position corresponding to fifth target
const int WAIT_POSITION        = TARGET_3_POSITION; // [encoder counts] Motor position corresponding to a wait position (when no targets are active)
const int LOWER_BOUND          = TARGET_1_POSITION; // [encoder counts] Position of the left end stop
const int UPPER_BOUND          = TARGET_4_POSITION; // [encoder counts] Position of the right end stop
const int TARGET_BAND          = 5; // [encoder counts] "Close enough" range when moving towards a target.
// Timing:
//const long  WAIT_TIME          = 0; // [microseconds] Time waiting for the target to drop.
                                      // TBD - implement a timer so if the target doesn't drop, the linkage moves to a different target
// VARIABLES:
//unsigned long startWaitTime; // [microseconds] System clock value

//** PID Controller  **//
// CONSTANTS:
const float KP                    = 0; // [Volt / encoder counts] P-Gain
const float KI                    = 0; // [Volt / (encoder counts * seconds)] I-Gain
const float KD                    = 0; // [Volt * seconds / encoder counts] D-Gain
const float SUPPLY_VOLTAGE        = 0; // [Volt] Supply voltage at the HBridge
const float FRICTION_COMP_VOLTAGE = 0; // [Volt] Voltage needed to overcome friction
// VARIABLES:
int desiredPosition  = 0; // [encoder counts] desired motor position
float positionError  = 0; // [encoder counts] Position error
float integralError  = 0; // [encoder counts * seconds] Integrated position error
float velocityError  = 0; // [encoder counts / seconds] Velocity error
float desiredVoltage = 0; // [Volt] Desired motor voltage
int   motorCommand   = 0; // [0-255] PWM signal sent to the motor
unsigned long executionDuration = 0; // [microseconds] Time between this and the previous loop execution.  Variable used for integrals and derivatives
unsigned long lastExecutionTime = 0; // [microseconds] System clock value at the moment the loop was started the last time

//** Pin assignment: **//
// CONSTANTS:
const int PIN_NR_ENCODER_A        = 2;  // Never change these, since the interrupts are attached to pins 2 and 3
const int PIN_NR_ENCODER_B        = 3;  // Never change these, since the interrupts are attached to pins 2 and 3
const int PIN_NR_ON_OFF_SWITCH    = 5;  // Connected to toggle switch (turns mechanism on and off)
const int PIN_NRL_LIMIT_SWITCH    = 8;  // Connected to limit switch (mechanism calibration)
const int PIN_NR_PWM_OUTPUT       = 11; // Connected to H Bridge (controls motor speed)
const int PIN_NR_PWM_DIRECTION_1  = 12; // Connected to H Bridge (controls motor direction)
const int PIN_NR_PWM_DIRECTION_2  = 13;  // Connected to H Bridge (controls motor direction)
const int PIN_PROXSENSE1          = A0; // Connected to proximity sensor 1
const int PIN_PROXSENSE2          = A1; // Connected to proximity sensor 2
const int PIN_POTENTIOMETER       = A2; // Connected to potentiometer used to test target positions


// Add this line of code if you want to use two limit switches; const int PIN_NRL_LIMIT_SWITCH_2  = 11
// ^KEEP IN MIND THAT YOU HAVE TO ADD CODE DOWNSTREAM (FOR EXAMPLE YOU NEED TO ADD THIS VARIABLE IN THE DECLARATION SECTION

// End of CONSTANTS AND GLOBAL VARIABLES


//////////////////////////////////////////////////////////////////////////////////////////
// The setup() function is called when a sketch starts. Use it to initialize variables, //
// pin modes, start using libraries, etc. The setup function will only run once, after  //
// each powerup or reset of the Arduino board:                                          //
//////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  // Declare which digital pins are inputs and which are outputs:
  pinMode(PIN_NR_ENCODER_A,        INPUT_PULLUP);
  pinMode(PIN_NR_ENCODER_B,        INPUT_PULLUP);
  pinMode(PIN_NR_ON_OFF_SWITCH,    INPUT);
  pinMode(PIN_NRL_LIMIT_SWITCH,    INPUT);
  pinMode(PIN_PROXSENSE1,          INPUT); 
  pinMode(PIN_PROXSENSE2,          INPUT);
  pinMode(PIN_NR_PWM_OUTPUT,       OUTPUT);
  pinMode(PIN_NR_PWM_DIRECTION_1,  OUTPUT);
  pinMode(PIN_NR_PWM_DIRECTION_2,  OUTPUT);
  pinMode(PIN_POTENTIOMETER,       INPUT);
  
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

  // Set initial output to the motor to 0
  analogWrite(PIN_NR_PWM_OUTPUT, 0);
}
// End of function setup()


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
  // The state machine:
  switch (state) {
    //****************************************************************************//
    // In the CALIBRATE state, we move the mechanism to a position outside of the 
    // work space (towards the limit switch).  Once the limit switch is on and 
    // the motor has stopped turning, we know that we are against the end stop
    case CALIBRATE:
      // We don't have to do anything here since this state is only used to set
      // a fixed output voltage.  This happens further below.
      
      // Decide what to do next:
      if (digitalRead(PIN_NRL_LIMIT_SWITCH)==HIGH && motorVelocity==0) { 
        // We reached the endstop.  Update the motor position to the limit:
        // (NOTE: If the limit switch is on the right, this must be UPPER_BOUND)
        motorPosition = LOWER_BOUND;  
        // Reset the error integrator:
        integralError = 0;
        // Calibration is finalized. Transition into DETERMINE_ACTIVE_TARGETS state
        Serial.println("State transition from CALIBRATE to DETERMINE_ACTIVE_TARGETS");
        state = DETERMINE_ACTIVE_TARGETS;
      } 
      // Otherwise we continue calibrating
      break;

    //****************************************************************************//
    // In the DETERMINE_ACTIVE_TARGETS state, we read the proximity sensors 10 times each, average 
    // values to reduce noise, then use them to determine which targets are active.
    // We then select an active target to move toward based on which targets are active.
    // We default to TARGET_POSITION_3 if no targets are active.
    case DETERMINE_ACTIVE_TARGETS:
      // Reset active targets array to all UNSURE
      for (int i = 0; i < sizeof(activeTargets)/sizeof(int); i++) {
        activeTargets[i] = UNSURE;
      }
      // Reset activeTargetPosition to UNSURE
      activeTargetPosition = UNSURE;
      
      // Read and average 20 photosensor values
      for (int i = 0; i < sizeof(proximitySense1Values)/sizeof(int); i++) {
//        Serial.println(analogRead(PIN_PROXSENSE1));
        proximitySense1Values[i] = analogRead(PIN_PROXSENSE1);
      }
      for (int i = 0; i < sizeof(proximitySense2Values)/sizeof(int); i++) {
        proximitySense2Values[i] = analogRead(PIN_PROXSENSE2);
      }
      proximitySense1Avg = average(proximitySense1Values, sizeof(proximitySense1Values)/sizeof(int));
      proximitySense2Avg = average(proximitySense2Values, sizeof(proximitySense2Values)/sizeof(int));


      // Compare proximitySense1Avg and proximitySense2Avg to thresholds to determine which targets are active
      if (proximitySense1Avg > TARGET1_PROXIMITYSENSE1_MIN) {
        activeTargets[TARGET1] = ACTIVE;    // Target 1 is active
        activeTargetPosition = TARGET_1_POSITION;
      }
      else if (proximitySense1Avg > TARGET2_PROXIMITYSENSE1_MIN && proximitySense1Avg < TARGET2_PROXIMITYSENSE1_MAX) {
        activeTargets[TARGET1] = INACTIVE;  // Target 1 is inactive
        activeTargets[TARGET2] = ACTIVE;    // Target 2 is active
        activeTargetPosition = TARGET_2_POSITION;
      }
      else if (proximitySense1Avg > TARGET3_PROXIMITYSENSE1_MIN && proximitySense1Avg < TARGET3_PROXIMITYSENSE1_MAX) {
        activeTargets[TARGET1] = INACTIVE;  // Target 1 is inactive
        activeTargets[TARGET2] = INACTIVE;  // Target 2 is inactive
        activeTargets[TARGET3] = ACTIVE;    // Target 3 is active
        activeTargetPosition = TARGET_3_POSITION;
      }
      else if (proximitySense1Avg < TARGET3_PROXIMITYSENSE1_MIN) {
        activeTargets[TARGET1] = INACTIVE;  // Target 1 is inactive
        activeTargets[TARGET2] = INACTIVE;  // Target 2 is inactive
        activeTargets[TARGET3] = INACTIVE;  // Target 3 is inactive
      }
      if (proximitySense2Avg > TARGET5_PROXIMITYSENSE2_MIN) {
        activeTargets[TARGET5] = ACTIVE;    // Target 5 is active
        // If activeTargetPosition has not already been changed, update it
        if (activeTargetPosition == -1) {
          activeTargetPosition = TARGET_5_POSITION;
        }
      }
      else if (proximitySense2Avg > TARGET4_PROXIMITYSENSE2_MIN && proximitySense2Avg < TARGET4_PROXIMITYSENSE2_MAX) {
        activeTargets[TARGET5] = INACTIVE;  // Target 5 is inactive
        activeTargets[TARGET4] = ACTIVE;    // Target 4 is active
        // If activeTargetPosition has not already been changed, update it
        if (activeTargetPosition == -1) {
          activeTargetPosition = TARGET_4_POSITION;
        }
      }
      else if (proximitySense2Avg > TARGET3_PROXIMITYSENSE2_MIN && proximitySense2Avg < TARGET3_PROXIMITYSENSE2_MAX) {
        activeTargets[TARGET5] = INACTIVE;  // Target 5 is inactive
        activeTargets[TARGET4] = INACTIVE;  // Target 4 is inactive
        activeTargets[TARGET3] = ACTIVE;    // Target 3 is active
      }
      else if (proximitySense2Avg < TARGET3_PROXIMITYSENSE2_MIN) {
        activeTargets[TARGET5] = INACTIVE;  // Target 5 is inactive
        activeTargets[TARGET4] = INACTIVE;  // Target 4 is inactive
        activeTargets[TARGET3] = INACTIVE;  // Target 3 is inactive
      }
      if (proximitySense1Avg < TARGETNONE_PROXIMITYSENSE1_MAX && proximitySense2Avg < TARGETNONE_PROXIMITYSENSE2_MAX) {
        for (int i = 0; i < sizeof(activeTargets)/sizeof(int); i++) {
          activeTargets[i] = INACTIVE;  // All targets are inactive
        }
        activeTargetPosition = WAIT_POSITION; // Make linkage go to wait position if no targets are active
      }

      // Read potentiometer to determine which target is active USED FOR TESTING PURPOSES ONLY
//      potValue = analogRead(PIN_POTENTIOMETER);
//      Serial.print("potValue: ");
//      Serial.println(potValue);
//      if (potValue > 100 && potValue < 200) {
//        activeTargetPosition = TARGET_1_POSITION;
//      }
//      else if (potValue > 300 && potValue < 400) {
//        activeTargetPosition = TARGET_2_POSITION;
//      }
//      else if (potValue > 500 && potValue < 600) {
//        activeTargetPosition = TARGET_3_POSITION;
//      }
//      else if (potValue > 700 && potValue < 800) {
//        activeTargetPosition = TARGET_4_POSITION;
//      }
//      else if (potValue > 900 && potValue < 1000) {
//        activeTargetPosition = TARGET_5_POSITION;
//      }
      
      // Proceed to MOVE_TO_TARGET if we have a valid activeTargetPosition:
      if (activeTargetPosition != UNSURE) {
        state = MOVE_TO_TARGET;
      }
      // Otherwise, we stay in DETERMINE_ACTIVE_TARGETS
      break;

    //****************************************************************************//
    // In the MOVE_TO_TARGET state, we select an active target and move toward it, or
    // move toward Target 3 (a default position) if there is no active target
    case MOVE_TO_TARGET:
      desiredPosition = activeTargetPosition;
      if (motorPosition <= activeTargetPosition + TARGET_BAND && motorPosition >= activeTargetPosition - TARGET_BAND) {
        state = DETERMINE_ACTIVE_TARGETS;
      }
      break;

    //****************************************************************************//
    // We should never reach the next bit of code, which would mean that the state
    // we are currently in doesn't exist.  So if it happens, throw an error and 
    // stop the program:
    default: 
      Serial.println("Statemachine reached at state that it cannot handle.  ABORT!!!!");
      Serial.print("Found the following unknown state: ");
      Serial.println(state);
      while (1); // infinite loop to halt the program
    break;
  }
  // End of the state machine.
  //******************************************************************************//

  //******************************************************************************//
  // Recalibrate if we are in the leftmost position
  if (digitalRead(PIN_NRL_LIMIT_SWITCH)==HIGH && motorVelocity==0) { 
        // We reached the endstop.  Update the motor position to the limit:
        // (NOTE: If the limit switch is on the right, this must be UPPER_BOUND)
        motorPosition = LOWER_BOUND;  
        // Reset the error integrator:
        integralError = 0;
  } 
  
 
  //******************************************************************************//
  // Position Controller
  if (digitalRead(PIN_NR_ON_OFF_SWITCH)==HIGH) {
    // If the toggle switch is on, run the controller:

    //** PID control: **//  
    // Compute the position error [encoder counts]
    positionError = desiredPosition - motorPosition;
    // Compute the integral of the position error  [encoder counts * seconds]
    integralError = integralError + positionError * (float)(executionDuration) / 1000000; 
    // Compute the velocity error (desired velocity is 0) [encoder counts / seconds]
    velocityError = 0 - motorVelocity;
    // This is the actual controller function that uses the error in 
    // position and velocity and the integrated error and computes a
    // desired voltage that should be sent to the motor:
    desiredVoltage = KP * positionError +  
                     KI * integralError +
                     KD * velocityError;
 
    //** Feedforward terms: **//
    // Compensate for friction.  That is, if we now the direction of 
    // desired motion, add a base command that helps with moving in this
    // direction:
    if (positionError < -5) {
      desiredVoltage = desiredVoltage - FRICTION_COMP_VOLTAGE;
    }
    if (positionError > +5) {
      desiredVoltage = desiredVoltage + FRICTION_COMP_VOLTAGE;
    }

    // Anti-Wind-Up
    if (abs(desiredVoltage)>SUPPLY_VOLTAGE) {
      // If we are already saturating our output voltage, it does not make
      // sense to keep integrating the error (and thus ask for even higher
      // and higher output voltages).  Instead, stop the integrator if the 
      // output saturates. We do this by reversing the summation at the 
      // beginning of this function block:
      integralError = integralError - positionError * (float)(executionDuration) / 1000000; 
    }
    // End of 'if(onOffSwitch==HIGH)'
    
    // Override the computed voltage during calibration.  In this state, we simply apply a 
    // fixed voltage to move against one of the end-stops.
    if (state==CALIBRATE) {
      desiredVoltage = CALIBRATION_VOLTAGE      
    }
  } else { 
    // Otherwise, the toggle switch is off, so do not run the controller, 
    // stop the motor...
    desiredVoltage = 0; 
    // .. and reset the integrator of the error:
    integralError = 0;
    // Produce some debugging output:
    Serial.println("The toggle switch is off.  Motor Stopped.");
  } 
  // End of  else onOffSwitch==HIGH
  
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
  } else {
    // ... otherwise turn backward:
    digitalWrite(PIN_NR_PWM_DIRECTION_1,HIGH); // rotate backward
    digitalWrite(PIN_NR_PWM_DIRECTION_2,LOW);  // rotate backward
  }
  // End of Position Controller
  //*********************************************************************//
  
  // Print out current controller state to Serial Monitor.
  printStateToSerial();
}
// End of main loop
//***********************************************************************//


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
// End of function updateMotorPosition()


//////////////////////////////////////////////////////////////////////
// This function sends a status of the controller to the serial     //
// monitor.  Each character will take 85 microseconds to send, so   //
// be selective in what you write out:                              //
//////////////////////////////////////////////////////////////////////
void printStateToSerial() {
  //*********************************************************************//
  // Send a status of the controller to the serial monitor.  
  // Each character will take 85 microseconds to send, so be selective
  // in what you write out:

  //Serial.print("State Number:  [CALIBRATE = 1; DETERMINE_ACTIVE_TARGETS = 2; MOVE_TO_TARGET = 3]: ");
  Serial.print("State#: "); 
  Serial.print(state);

  //Serial.print("Power switch [on/off]: ");
  //Serial.print("  PWR: "); 
  //Serial.print(digitalRead(PIN_NR_ON_OFF_SWITCH));

  //Serial.print("      Motor Position [encoder counts]: ");
  Serial.print("  MP: "); 
  Serial.print(motorPosition);

  //Serial.print("      Motor Velocity [encoder counts / seconds]: ");
  Serial.print("  MV: "); 
  Serial.print(motorVelocity);

  //Serial.print("      Encoder Status [4 bit value]: ");
  //Serial.print("  ES: "); 
  //Serial.print(encoderStatus);

  //Serial.print("      Target Position [encoder counts]: ");
  Serial.print("  DP: "); 
  Serial.print(desiredPosition);

  //Serial.print("      Position Error [encoder counts]: ");
  Serial.print("  PE: "); 
  Serial.print(positionError);

  //Serial.print("      Integrated Error [encoder counts * seconds]: ");
  Serial.print("  IE: "); 
  Serial.print(integralError);

  //Serial.print("      Velocity Error [encoder counts / seconds]: ");
  Serial.print("  VE: "); 
  Serial.print(velocityError);

  //Serial.print("      Desired Output Voltage [Volt]: ");
  Serial.print("  DV: "); 
  Serial.print(desiredVoltage);
  
  //Serial.print("      Motor Command [0-255]: ");
  //Serial.print("  MC: "); 
  //Serial.print(motorCommand);

  //Serial.print("      Execution Duration [microseconds]: ");
  //Serial.print("  ED: "); 
  //Serial.print(executionDuration);

  //Serial.print("      Active Targets [-1,0,1]: ");
  Serial.print("  AT: ");
  for (int i = 0; i < sizeof(activeTargets)/sizeof(int); i++) {
    Serial.print(activeTargets[i]);
    Serial.print(" ");
  }

//  Serial.print("  PS2: ");
//  Serial.print(proximitySense2Avg);
  
  // ALWAYS END WITH A NEWLINE.  SERIAL MONITOR WILL CRASH IF NOT
  Serial.println(); // new line
}
// End of Serial Out

//////////////////////////////////////////////////////////////////////
// This function returns the average of the integer array           //
// pointed to by array_ptr and of length len.                       //
//////////////////////////////////////////////////////////////////////
float average (int * array_ptr, int len) {
  long sum = 0L;
  for (int i = 0; i < len; i++) {
    sum += array_ptr[i];
  }
  return ((float) sum)/len;
}
// End of average
