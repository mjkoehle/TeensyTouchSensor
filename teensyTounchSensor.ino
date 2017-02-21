/* Michael Koehle 01.30.2017
Send json packet whenever state of one of the two touch sensors is changed. 
Sensors are filtered using moving average. An initial calibration sets a threshold for
each sensor. This is equal to sensorMax + thresholdScale * (sensorMax-sensorMin)
ThresholdScale is a user set value

 */

 // MK read dip. 

/*user variables*/
int sourceIP = 100; //set the ip for this teensy
double thresholdScale = .3; // see above
int calTime = 5000;  //calibraton time on startup
int beatDelay = 30000; //interval between sending beat packets, in millisenconds

#include <SPI.h>
#include <Ethernet.h>
#include <ArduinoJson.h> //need arduinoJson library here: https://github.com/bblanchon/ArduinoJson
#include "mac.h" //local import


#define s0 17
#define s1 16
#define s2 15
#define s3 14

/*network variables*/
const unsigned int resetPin = 9; //Ethernet Reset Pin
//extern uint8_t mac[6];
byte macB[] = {0x04, 0xE9, 0xE5, 0x03, 0x5F, 0x44};
//04:E9:E5:03:5F:10 [Teensy A]
//04:E9:E5:03:5F:44 [Teensy B]

IPAddress localIp(192, 168, 0, sourceIP);
IPAddress remoteIp(192, 168, 0 , 3);
unsigned int serverPort = 3335; //change variable name
unsigned int localPort = 7777;
EthernetUDP udp;
IPAddress localCheck;
EthernetClient client;

/* PANEL */
int currentModule;

/*packet variables*/
String jsonString1 = "";
String jsonString2 = "";
String touchStart = "start_touch";
String touchStop = "end_touch";
String touchBeat = "touch_beat";
String flavor1 = touchBeat;
String flavor2 = touchBeat;
String choice1 = "left"; // color for pin1
String choice2 = "right"; //color for pin2

/*led variables*/
int redPin= 19; //red is 19
int greenPin = 20; //green is 20
int bluePin = 21;
byte redValue = LOW;
byte greenValue = LOW;
byte blueValue = LOW;

/*moving average variables*/
const int numReadings = 5; // Number of samples to average
double readings1[numReadings];      // the readings from the analog input
double readings2[numReadings];
int readIndex = 0;              // the index of the current reading
double total1 = 0;                  // the running total
double average1 = 0;               // averaged readings
double total2 = 0;
double average2 = 0;

int inputPin1 = A8;   //  touch sensor1
int inputPin2 = A9;   // touch sensor2 

/* messaging logic variables*/
int triggerTime1 = 0;
int triggerTime2 = 0;
int currentBeat = 0; 
boolean trigger1 = false;
boolean trigger2 = false;
boolean sendMsg1 = false;
boolean sendMsg2 = false;
double thresholds[2];
double threshold1 = 0;
double threshold2 = 0;

void setup() {
  
  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings1[thisReading] = 0;
    readings2[thisReading] = 0;
  }

  Serial.begin(38400);
//  while (!Serial) {
//    ; // wait for serial port to connect. Needed for native USB port only
//  }

 
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT); 
  
  digitalWrite(redPin, 1);   // turn the LED on during calibration
  digitalWrite(greenPin, 1);  
  
  calibrate(&thresholds[0]); //calibrate
  
  digitalWrite(redPin, 0);   // turn the LED off after calibration
  digitalWrite(greenPin, 0);   

  //connection:
  currentModule = readDIPAddress();    //Read address from DIP
  localIp = IPAddress(192, 168, 0, currentModule + 100); //101-103 are touch sensor. 104-1011 ceiling
  getMAC(); //assigns MAC address to variable mac
   //Reset Ethernet Module
  resetEthernet();
  Ethernet.begin(mac, localIp);
  udp.begin(localPort);
  //client.connect(remoteIp, serverPort);  //MK this is for testing connection to a specific IP. Is there a check for just ethernet connection?
  //client.connected //.maintain

  Serial.println(localIp);
  threshold1 = thresholds[0];
  threshold2 = thresholds[1];
  delay(1000);
  
  }


void loop() { 

  /* Moving Average*/
  // subtract the last reading:
  total1 = total1 - readings1[readIndex];
  total2 = total2 - readings2[readIndex];
  // read from the sensor:
  readings1[readIndex] = touchRead(inputPin1);
  readings2[readIndex] = touchRead(inputPin2);
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

  /*Message Triggering*/
  sendMsg1 = false;
  if (average1 >= threshold1) {
    if (trigger1 == false) { //if trigger1 is going from false to true, send msg to server
      sendMsg1 = true;
      redValue = HIGH;
    }
    triggerTime1 = triggerTime1 + 1;
    trigger1 = true; //trigger1 is on
    flavor1 = touchStart;
  }
  else {
    if (trigger1 == true) {//if trigger1 is going from true to false, send msg to server
      sendMsg1 = true;
      redValue = LOW;
    }
    triggerTime1 = 0;
    trigger1 = false;
    flavor1 = touchStop;
  }
  
  sendMsg2 = false;
  if (average2 >= threshold2) {
    if (trigger2 == false) { //if trigger2 is going from false to true, send msg to server
      sendMsg2 = true;
      greenValue = HIGH;
    }
    triggerTime2 = triggerTime2 + 1;
    trigger2 = true; //trigger1 is on
    flavor2 = touchStart;

  }
  else {
    if (trigger2 == true) {
      sendMsg2 = true;
      greenValue = LOW;
    }
    triggerTime2 = 0;
    trigger2 = false;
    flavor2 = touchStop;
  }
  
if (millis()-currentBeat > beatDelay) {
    sendMsg1 = true;
    sendMsg2 = true;
    flavor1 = touchBeat;
    flavor2 = touchBeat;
    currentBeat = millis();
  }

  /*visualization*/
  Serial.print(touchRead(inputPin1));
  Serial.print("\t");
  Serial.print(average1);
  Serial.print("\t");
   Serial.print(threshold1);
  Serial.print(touchRead(inputPin2));
  Serial.print("\t");
  Serial.print(average2);
  Serial.print("\t");
  Serial.println(threshold2);

  /* Json messaging */
  if (sendMsg1) {

      /*send packet*/
//      DynamicJsonBuffer jsonBuffer3;
      char jsonChar1[100];
      jsonString1 = "{\"source\": \"" + String(sourceIP) + "\",\"flavor\" :\""+ flavor1+"\",\"choice\" :\"" + choice1 + "\"}";
      jsonString1.toCharArray(jsonChar1,100);
      udp.beginPacket(remoteIp, serverPort);
      udp.write(jsonChar1);
      udp.println();
      udp.endPacket();
      
      /*write color*/
      digitalWrite(redPin, redValue);
      digitalWrite(bluePin, blueValue);
      digitalWrite(greenPin, greenValue);

  }
  
  if (sendMsg2) {

      /*send packet*/
      char jsonChar2[100];
      jsonString2 = "{\"source\": \"" + String(sourceIP) + "\",\"flavor\" :\""+ flavor1+"\",\"choice\" :\"" + choice1 + "\"}";
      jsonString2.toCharArray(jsonChar2,100);
      udp.beginPacket(remoteIp, serverPort);
      udp.write(jsonChar2);
      udp.println();
      udp.endPacket();

      /*write color*/
      digitalWrite(redPin, redValue);
      digitalWrite(bluePin, blueValue);
      digitalWrite(greenPin, greenValue);

  }

  delay(1);
 } 
 /*end Loop*/
  



/*CALIBRATION*/
void calibrate(double *thresholds)
{

  /*cal variables*/
  int touchTime = 1000; //calibration for touching both sensors

  int sensorValue1 = 0;         // the sensor value
  int sensorMin1 = 100000;        // minimum sensor value
  int sensorMax1 = 0;           // maximum sensor value
  int touchMax1 = 0;
  int sensorValue2 = 0;         // the sensor value
  int sensorMin2 = 100000;        // minimum sensor value
  int sensorMax2 = 0;           // maximum sensor value
  int touchMax2 = 0;

  double idle1 = 0;  //touch calibration variable. idle is halfway between max and min during not touched 
  double idle2 = 0;
  
  double threshold1 = 0;
  double threshold2 = 0;

  int minTimer1 = 0;
  int counter = 0;
  int flagMe = 1;
  int debugOn = 1;
  
  while (millis() < calTime + touchTime) {
    counter = counter+1;

    /* Moving Average*/
    // subtract the last reading:
    total1 = total1 - readings1[readIndex];
    total2 = total2 - readings2[readIndex];
    // read from the sensor:
    readings1[readIndex] = touchRead(inputPin1);
    readings2[readIndex] = touchRead(inputPin2);
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

    /*calculate min/maxes*/
    if (counter > numReadings*2 && millis() < calTime) { //give it some for moving average to take affect
      // record the maximum sensor value    
      if ( average1 > sensorMax1) {
        sensorMax1 =  average1;
      }
      
      // record the minimum sensor value
      if ( average1 < sensorMin1) {
        sensorMin1 =  average1;
      }

      // record the maximum sensor value
      if ( average2 > sensorMax2) {
        sensorMax2 =  average2;
      }

      // record the minimum sensor value
      if (average2 < sensorMin2) {
        sensorMin2 =  average2;
      }
    }
    else if  (millis() > calTime)  { //only used for touch calibration
      // record the maximum sensor value
      if (  average1 > touchMax1) {
        touchMax1 =  average1;
      }
      // record the maximum sensor value
      if (  average2 > touchMax2) {
        touchMax2 =  average2;
      }

    }
  }
  /*method 1, if calibration includes a touch*/
  idle1 = (sensorMax1 + sensorMin1) / 2;
  idle2 = (sensorMax2 + sensorMin2) / 2;
  thresholds[0] = idle1 + .5 * (touchMax1 - idle1); //calculate a threshold for trigger
  thresholds[1] = idle2 + .5 * (touchMax2 - idle2); //calculate a threshold for trigger

  /*method2: for calibration without touching sensor. */
  thresholds[0] = sensorMax1 + thresholdScale * (sensorMax1-sensorMin1); //calculate a threshold for trigger
  thresholds[1] = sensorMax2 + thresholdScale * (sensorMax2-sensorMin2); //calculate a threshold for trigger

//  delay(1);

}

void getMAC() {
  delay(1000);

  Serial.println("Reading MAC from hardware...");
  read_mac();

  Serial.print("MAC: ");
  print_mac();
  Serial.println();


  Serial.println("Finished.");

}

//Reset Ethernet Module
void resetEthernet() {

  pinMode(resetPin, OUTPUT);
  digitalWrite(resetPin, LOW);
  delayMicroseconds(10);
  //delay(1000);
  pinMode(resetPin, INPUT);

  Serial.println("Reset Done");
  delay(1000);
}

int readDIPAddress() {

  int address;
  // State of each switch (0 or 1)
  int s0state;
  int s1state;
  int s2state;
  int s3state;


  pinMode(s0, INPUT_PULLUP);
  pinMode(s1, INPUT_PULLUP);
  pinMode(s2, INPUT_PULLUP);
  pinMode(s3, INPUT_PULLUP);
  delay(20);

  s0state = digitalReadFast(s0);
  s1state = digitalReadFast(s1);
  s2state = digitalReadFast(s2);
  s3state = digitalReadFast(s3);

  bitWrite(address, 0, !s0state);
  bitWrite(address, 1, !s1state);
  bitWrite(address, 2, !s2state);
  bitWrite(address, 3, !s3state);


  Serial.print("DIP Address =>  ");
  Serial.println(address);
  //delay(2000);
  //    Serial.println(muxValue);

  return address;

}
