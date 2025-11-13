// Sketch for the toggle switch

// Set the initial values of variables
int onOffSwitch = 0;

const int onOffSwitchPin = 5;

void setup() {
  // Declare which digital pins are input and output
  pinMode(onOffSwitchPin, INPUT);

  // Begin serial communication for display of variable states
  Serial.begin(115200);
  Serial.println("Start");

}

void loop() {
  
  onOffSwitch = digitalRead(onOffSwitchPin);

  if (onOffSwitch == HIGH){
    Serial.println("Switch on");
  } else {
    Serial.println("Switch off");
  }

}
