// CHANGE THESE VALUES
// Definition of proximity sensor thresholds for each target:
const int PROXIMITYSENSE1_MAX = 598;
const int PROXIMITYSENSE2_MAX = 594;
const int PROXIMITYSENSE3_MAX = 607;
const int PROXIMITYSENSE4_MAX = 610;
const int PROXIMITYSENSE1_MIN = 105;
const int PROXIMITYSENSE2_MIN = 124;
const int PROXIMITYSENSE3_MIN = 118;
const int PROXIMITYSENSE4_MIN = 99;   
int PROX_MIN_MAX[4][2] = {{2000, 0}, {2000, 0}, {2000, 0}, {2000, 0}};
bool printed = false;

// DO NOT CHANGE ANYTHING BELOW THIS LINE
// Definition of target variables for array indexing
const int TARGET1  = 0;   // For indexing into activeTargets array, activeTargets[0] corresponds to Target 1
const int TARGET2  = 1;   // For indexing into activeTargets array, activeTargets[1] corresponds to Target 2
const int TARGET3  = 2;   // For indexing into activeTargets array, activeTargets[2] corresponds to Target 3
const int TARGET4  = 3;   // For indexing into activeTargets array, activeTargets[3] corresponds to Target 4
const int TARGET5  = 4;   // For indexing into activeTargets array, activeTargets[4] corresponds to Target 5

// VARIABLES:
// Create variables that read and average proximity sensor or potentiometer readings:
int proximitySense1Values[20]; // Holds 20 proximity sensor readings for averaging
int proximitySense2Values[20]; // Holds 20 proximity sensor readings for averaging
int proximitySense3Values[20]; // Holds 20 proximity sensor readings for averaging
int proximitySense4Values[20]; // Holds 20 proximity sensor readings for averaging

float proximitySenseAvgs[4]; // Holds most recent average of proximity sensor values

// Pin assignments
const int PIN_PROXSENSE1          = A0; // Connected to proximity sensor 1
const int PIN_PROXSENSE2          = A1; // Connected to proximity sensor 2
const int PIN_PROXSENSE3          = A2; // Connected to proximity sensor 3
const int PIN_PROXSENSE4          = A3; // Connected to proximity sensor 4


void setup() {
  // put your setup code here, to run once:
  // Declare proximity sensor pins as inputs
  pinMode(PIN_PROXSENSE1,          INPUT); 
  pinMode(PIN_PROXSENSE2,          INPUT);
  pinMode(PIN_PROXSENSE3,          INPUT); 
  pinMode(PIN_PROXSENSE4,          INPUT);

  // Begin serial communication for monitoring.
  Serial.begin(115200);
  Serial.println("Start Executing Program.");
}


void loop() {
  // put your main code here, to run repeatedly:

  // Read and average 20 photosensor values
  for (int i = 0; i < sizeof(proximitySense1Values)/sizeof(int); i++) {
    proximitySense1Values[i] = analogRead(PIN_PROXSENSE1);
  }
  for (int i = 0; i < sizeof(proximitySense2Values)/sizeof(int); i++) {
    proximitySense2Values[i] = analogRead(PIN_PROXSENSE2);
  }
  for (int i = 0; i < sizeof(proximitySense3Values)/sizeof(int); i++) {
    proximitySense3Values[i] = analogRead(PIN_PROXSENSE3);
  }
  for (int i = 0; i < sizeof(proximitySense4Values)/sizeof(int); i++) {
    proximitySense4Values[i] = analogRead(PIN_PROXSENSE4);
  }

  // Compute running average for each prox sensor
  proximitySenseAvgs[0] = average(proximitySense1Values, sizeof(proximitySense1Values)/sizeof(int));
  proximitySenseAvgs[1] = average(proximitySense2Values, sizeof(proximitySense2Values)/sizeof(int));
  proximitySenseAvgs[2] = average(proximitySense3Values, sizeof(proximitySense3Values)/sizeof(int));
  proximitySenseAvgs[3] = average(proximitySense4Values, sizeof(proximitySense4Values)/sizeof(int));

  // Print averages to serial output
  if (millis() < 44500) { 
    Serial.print("PROX_1:");
    Serial.print(proximitySenseAvgs[0]);
    Serial.print(",PROX_2:");
    Serial.print(proximitySenseAvgs[1]);
    Serial.print(",PROX_3:");
    Serial.print(proximitySenseAvgs[2]);
    Serial.print(",PROX_4:");
    Serial.println(proximitySenseAvgs[3]);
  }

  // Update new min and max
  for (int i = 0; i < 4; i++) {
    PROX_MIN_MAX[i][0] = min(PROX_MIN_MAX[i][0], proximitySenseAvgs[i]);
    PROX_MIN_MAX[i][1] = max(PROX_MIN_MAX[i][1], proximitySenseAvgs[i]);
  }

  if (millis() > 44500 && !printed) { 
    printed = true;
    for (int i = 1; i < 5; i++) {
        Serial.print("PROX ");
        Serial.print(i);
        Serial.print(" MIN: ");
        Serial.print(PROX_MIN_MAX[i-1][0]);
        Serial.print("\tPROX ");
        Serial.print(i);
        Serial.print(" MAX: ");
        Serial.println(PROX_MIN_MAX[i-1][1]);
    }
  }


}
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

