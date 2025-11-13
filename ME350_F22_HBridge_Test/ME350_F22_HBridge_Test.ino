/* ME350 H-Bridge Test
   Updated 10/13/2022 */


const int PIN_NR_ON_OFF_SWITCH    = 5;  // Connected to toggle switch (turns mechanism on and off)
const int PIN_NR_PWM_OUTPUT       = 11; // Connected to H Bridge (controls motor speed)
const int PIN_NR_PWM_DIRECTION_1  = 12; // Connected to H Bridge (controls motor direction)
const int PIN_NR_PWM_DIRECTION_2  = 13; // Connected to H Bridge (controls motor direction)
int H_Bridge_command = 0;             // [0-255] PWM signal sent to the H-bridge

void setup() {
  Serial.begin(115200);
  Serial.println("Start\n");
  
  pinMode(PIN_NR_PWM_OUTPUT,       OUTPUT);
  pinMode(PIN_NR_PWM_DIRECTION_1,  OUTPUT);
  pinMode(PIN_NR_PWM_DIRECTION_2,  OUTPUT);
  pinMode(PIN_NR_ON_OFF_SWITCH,    INPUT);
}

void loop() {
  if(digitalRead(PIN_NR_ON_OFF_SWITCH)==HIGH){
    // If the toggle switch is on, provide voltage
    H_Bridge_command = 255;
  }
  else{
    H_Bridge_command = 0;
  }
  analogWrite(PIN_NR_PWM_OUTPUT, H_Bridge_command); // Command a duty cycle
  digitalWrite(PIN_NR_PWM_DIRECTION_1,LOW);  // rotate forward
  digitalWrite(PIN_NR_PWM_DIRECTION_2,HIGH); // rotate forward
  Serial.print(H_Bridge_command);
  Serial.print("\n");
}
