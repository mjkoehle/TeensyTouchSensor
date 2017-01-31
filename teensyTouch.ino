// Michael Koehle 01.30.2017
// Using Teensy 3.2's built in touch sensor
// Filtered using moving average

const int numReadings = 20; // Number of samples to average

int readings1[numReadings];      // the readings from the analog input
int readings2[numReadings]; 
int readIndex = 0;              // the index of the current reading
int total1 = 0;                  // the running total
int average1 = 0;               // averaged readings
int total2 = 0; 
int average2 = 0; 

int inputPin1 = A9;   //  touch pin 
int inputPin2 = A8;   // touch pin

void setup() {
  // initialize serial communication with computer:
  Serial.begin(38400);
  
  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings1[thisReading] = 0;
    readings2[thisReading] = 0;
  }
}

 
void loop() {
  // subtract the last reading:
  total1 = total1 - readings1[readIndex];
  total2 = total2 - readings2[readIndex];
  // read from the sensor:
  readings1[readIndex] = touchRead(A8);
  readings2[readIndex] = touchRead(A9);
  // add the reading to the total:
  total1 = total1 + readings1[readIndex];
  total2 = total2 + readings2[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average1 = total1 / numReadings;
  average2 = total2 / numReadings;
  // send it to the computer as ASCII digits
  Serial.print(average1);
  Serial.print("\t");
  Serial.println(average2);
  delay(1);        // delay in between reads for stability
}

   

