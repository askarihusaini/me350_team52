// Test of limit switch
// Set the initial values of variables
int limSwitch1 = 0;

// Set the initial values of variables
// Constants will not change
const int limSwitch1Pin = 8;

void setup() {
  // put your setup code here, to run once:

  // Begin serial communication for display of variable states
  Serial.begin(115200);
  Serial.println("Start");
  pinMode(limSwitch1Pin, INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

  // Read the limit switch and assign readings to the variable
  limSwitch1 = digitalRead(limSwitch1Pin);

  // Print results to the serial monitor
  Serial.print("/t"); // This prints a tab
  Serial.print("limSwitch 1=");
  Serial.print("\t"); // This prints a tab
  Serial.println(limSwitch1);

}
