#include "Settings_Pablo_Modified.h"
#include "Wire.h"

// Bluefruit Includes
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig_Pablo_Modified.h"

#define FACTORYRESET_ENABLE         0
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

#define TCAADDR 0x70
#define BLUETOOTH_MODE_PIN  8


SoftwareSerial bluetoothSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);
Adafruit_BluefruitLE_UART ble(bluetoothSerial, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);

// Initialize type for RGB Sensors
Adafruit_TCS34725_modified tcs0;
Adafruit_TCS34725_modified tcs1;
Adafruit_TCS34725_modified tcs2;
Adafruit_TCS34725_modified tcs3;
Adafruit_TCS34725_modified tcs4;
Adafruit_TCS34725_modified tcs5;

char message[LENGTH];
int extensionLength;
int extensionSetting;

bool test = false;
bool runSensors = false;

static String inputBuffer;
static String fullBuffer = "";
static long lastRXTime = millis();
long lastTempTime = millis();

void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

void setup() {
  Serial.begin(9600);
  if (!ble.begin(VERBOSE_MODE)){
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
    }
  
  extensionSetting = 0;
  extensionLength = FULL_EXTENSION; // Used to determine how much the actuator extend
 
  //Serial.println(F("Setup Pins")); setupPins(); // Setups the non-bluetooth pins
  Serial.println(F("Setup Sensors")); setupSensors(); // Setups the RGB Sensors // Loose pin somewhere in setupSensors()
  
  // Process to setup bluetooth
  if(!test) {
    Serial.println(F("Setting up Bluetooth..."));
    FactoryResetBluetooth();
    //pinMode(8, OUTPUT); digitalWrite(8, HIGH); // Set MODE on Bluefruit to HIGH == Command Mode
    Serial.println(F("Finished Bluetooth Setup"));
  }
}

void loop() {
  if(test)
    runSensorTests();
  else{
    processReceivedData();
    if (fullBuffer != "") {
      chooseMode(fullBuffer);
      fullBuffer = ""; // Empties the buffer
    }
  }
}

void setupPins() {
  // LED Pins
  pinMode(BOARD_LED, OUTPUT);
  digitalWrite(BOARD_LED, LOW);
  
  // Motor Pins
  pinMode(MOTOR, OUTPUT);
  analogWrite(MOTOR, NO_EXTENSION);
}

void FactoryResetBluetooth() {
  if (FACTORYRESET_ENABLE){
    Serial.println(F("Performing a factory reset: "));
    if (!ble.factoryReset()){
      error(F("Couldn't factory reset"));
    }
  }
  
  ble.echo(false); // Disables command echo from Bluefruit
  //Serial.println("Requesting Bluefruit info:");
  //ble.info(); /* Print Bluefruit information */
  ble.verbose(false); // disables debug info
  
  if (ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION))
  {
    // Change Mode LED Activity
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }
}

void setupSensors() {
  //Sets up RGB on MUX Channel 0
  Serial.println("Setting up Sensor 0");
  delay(500);
  tcaselect(0);
  tcs0 = Adafruit_TCS34725_modified(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
  tcs0.begin();

  //Sets up RGB on MUX Channel 1
  Serial.println("Setting up Sensor 1");
  delay(500);
  tcaselect(1);
  tcs1 = Adafruit_TCS34725_modified(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
  tcs1.begin();

  //Sets up RGB on MUX Channel 2
  Serial.println("Setting up Sensor 2");
  delay(500);
  tcaselect(2);
  tcs2 = Adafruit_TCS34725_modified(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
  tcs2.begin();

  //Sets up RGB on MUX Channel 3
  Serial.println("Setting up Sensor 3");
  delay(500);
  tcaselect(3);
  tcs3 = Adafruit_TCS34725_modified(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
  tcs3.begin();

  //Sets up RGB on MUX Channel 4
  Serial.println("Setting up Sensor 4");
  delay(500);
  tcaselect(4);
  tcs4 = Adafruit_TCS34725_modified(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
  tcs4.begin();

  //Sets up RGB on MUX Channel 5
  Serial.println("Setting up Sensor 5");
  delay(500);
  tcaselect(5);
  tcs5 = Adafruit_TCS34725_modified(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
  tcs5.begin();

  Wire.begin(); 
}

void processReceivedData() {
  ble.println("AT+BLEUARTRX");
  ble.readline();
  
  // Check if there is data in the buffer
  if (strcmp(ble.buffer, "OK") == 0) {
    // no data
    return;
  }
  
  // Some data was found, its in the buffer
  Serial.print(F("[Recv] ")); Serial.println(ble.buffer);
  inputBuffer = ble.buffer;
  inputBuffer.trim(); // Remove \n\r from end.
  fullBuffer += inputBuffer;
  inputBuffer = "";
  ble.waitForOK();
}


void chooseMode(String mode) {
  if(mode != NULL) {
    Serial.print(F("Mode: "));
    Serial.println(mode);
  }
  switch (mode[0])
  {
  case 'B': // For beginning the stream (ASCII B)
    Serial.println(F("Begin Running the sensors."));
    int sensor;
    sensor = mode[1] - '0'; // Convert character to proper int
    if((sensor >= 0) && (sensor <= 9)) {
      readSensor(sensor);
    }
    break;
    
  case 'T':
    Serial.println(F("Run Tests"));
    break;
    
  case 'R': // Return fluid
    Serial.println(F("Return Fluid"));
    extensionSetting = 0;
    setExtension(extensionSetting);
    drawFluid();
    break;
  
  case 'S':
    Serial.println(F("Set Extension, Draw Fluid"));
    //Thank you C++ for not providing anyway to nicely convert between chars and ints...
    int digit2;
    int digit1;
    int digit0;
    
    digit2 = mode[1] - '0'; // Convert character to proper int
    digit1 = mode[2] - '0'; // Convert character to proper int
    digit0 = mode[3] - '0'; // Convert character to proper int
    
    extensionSetting = extensionSetting + (100*digit2 + 10*digit1 + 1*digit0);
    
    if((extensionSetting >= 1) && (extensionSetting <= 100)) {
      setExtension(extensionSetting);
    }
    else if(extensionSetting > 100) {
      extensionSetting = 100;
      setExtension(extensionSetting);
    }
    drawFluid();
    break;
  
  default:
    break;
  }
}

void readSensor(int sensor) {
  uint16_t alpha, red, green, blue;
  int sensorLED;
 
  digitalWrite(BOARD_LED, HIGH);
  tcaselect(sensor);
  switch(sensor){
    case 0:
      tcs0.enable();
      tcs0.getRawData(&red, &green, &blue, &alpha);
      break;
    case 1:
      tcs1.enable();
      tcs1.getRawData(&red, &green, &blue, &alpha);
      break;
    case 2:
      tcs2.enable();
      tcs2.getRawData(&red, &green, &blue, &alpha);
      break;
    case 3:
      tcs3.enable();
      tcs3.getRawData(&red, &green, &blue, &alpha);
      break;
    case 4:
      tcs4.enable();
      tcs4.getRawData(&red, &green, &blue, &alpha);
      break;
    case 5:
      tcs5.enable();
      tcs5.getRawData(&red, &green, &blue, &alpha);
      break;
  }
  digitalWrite(BOARD_LED, LOW);
// Check if any zeros are present, 1st try
  if(!(alpha)){
    Serial.print(F("C:\t")); Serial.print(alpha);Serial.print(F("\n"));
    Serial.print(F("Reading zero alpha. 1st re-attempt\n"));
    switch(sensor){
    case 0:
      tcs0.enable();
      tcs0.getRawData(&red, &green, &blue, &alpha);
      break;
    case 1:
      tcs1.enable();
      tcs1.getRawData(&red, &green, &blue, &alpha);
      break;
    case 2:
      tcs2.enable();
      tcs2.getRawData(&red, &green, &blue, &alpha);
      break;
    case 3:
      tcs3.enable();
      tcs3.getRawData(&red, &green, &blue, &alpha);
      break;
    case 4:
      tcs4.enable();
      tcs4.getRawData(&red, &green, &blue, &alpha);
      break;
    case 5:
      tcs5.enable();
      tcs5.getRawData(&red, &green, &blue, &alpha);
      break;
      }
  }
// Check if any zeros are present, 2nd try
  if(!(alpha)){
    Serial.print(F("C:\t")); Serial.print(alpha);Serial.print(F("\n"));
    Serial.print(F("Reading zero alpha. 2nd re-attempt\n"));
    switch(sensor){
    case 0:
      tcs0.enable();
      tcs0.getRawData(&red, &green, &blue, &alpha);
      break;
    case 1:
      tcs1.enable();
      tcs1.getRawData(&red, &green, &blue, &alpha);
      break;
    case 2:
      tcs2.enable();
      tcs2.getRawData(&red, &green, &blue, &alpha);
      break;
    case 3:
      tcs3.enable();
      tcs3.getRawData(&red, &green, &blue, &alpha);
      break;
    case 4:
      tcs4.enable();
      tcs4.getRawData(&red, &green, &blue, &alpha);
      break;
    case 5:
      tcs5.enable();
      tcs5.getRawData(&red, &green, &blue, &alpha);
      break;
      }
  }
   // Conversion of ARGB values to hex
   String astring = String(alpha,HEX);
   int astring_append = 4 - astring.length();
   switch(astring_append){
    case 0:
      break;
    case 1:
      astring = "0" + astring;
      break;
    case 2:
      astring = "00" + astring;
      break;
    case 3:
      astring = "000" + astring;
      break;
//    case 4:
//      astring = "0000" + astring;
//      break;
    }
   String rstring = String(red,HEX);
   int rstring_append = 4 - rstring.length();
   switch(rstring_append){
    case 0:
      break;
    case 1:
      rstring = "0" + rstring;
      break;
    case 2:
      rstring = "00" + rstring;
      break;
    case 3:
      rstring = "000" + rstring;
      break;
//    case 4:
//      rstring = "0000" + rstring;
//      break;
    }
   String gstring = String(green,HEX);
   int gstring_append = 4 - gstring.length();
   switch(gstring_append){
    case 0:
      break;
    case 1:
      gstring = "0" + gstring;
      break;
    case 2:
      gstring = "00" + gstring;
      break;
    case 3:
      gstring = "000" + gstring;
      break;
//    case 4:
//      gstring = "0000" + gstring;
//      break;
    }
   String bstring = String(blue,HEX);
   int bstring_append = 4 - bstring.length();
   switch(bstring_append){
    case 0:
      break;
    case 1:
      bstring = "0" + bstring;
      break;
    case 2:
      bstring = "00" + bstring;
      break;
    case 3:
      bstring = "000" + bstring;
      break;
//    case 4:
//      bstring = "0000" + bstring;
//      break;
    }

   // Concatenate strings into final message
   String messg = sensor + astring+rstring+gstring+bstring;

  // Send values over bluetooth to app
  if (!test){
  sendHEX(messg);
  Serial.println(F("Data Information:"));
  Serial.print(F("Sensor: ")); Serial.print(sensor);
  Serial.print(F("\tA: ")); Serial.print(alpha);
  Serial.print(F("\tR: ")); Serial.print(red);
  Serial.print(F("\tG: ")); Serial.print(green);
  Serial.print(F("\tB: ")); Serial.println(blue);
  Serial.println("------------------------------------");
  }
  else{
    Serial.println(sensor);
    Serial.println(messg);
    Serial.println("------------------------------------");
    }
}

void sendHEX(String message_to_send) {
  Serial.print(F("Transmitting:\t")); Serial.println(message_to_send);
  ble.print("AT+BLEUARTTX=");
  ble.println(message_to_send);
  }


void send(char color, uint16_t val) {
  message[0] = color;31;
  dtostrf(val, sizeof(val), 0, &message[1]);
  Serial.print(F("Transmitting:\t")); Serial.println(message);
  ble.print("AT+BLEUARTTX=");
  ble.println(message);
  memset(&message[0], 0, sizeof(message));//deletes message
}

void sendSensor(int sensor) {
  Serial.print(F("Transmitting:\t")); Serial.println(sensor);
  ble.print("AT+BLEUARTTX=");
  ble.println(sensor);
 }

void drawFluid() {
  analogWrite(MOTOR, extensionLength);  
}

void returnFluid() {
  analogWrite(MOTOR, NO_EXTENSION);
}

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void setExtension(int extensionSetting) {
  extensionLength = (int)round(((float)extensionSetting * (float)FULL_EXTENSION)/100.0);
}

void runSensorTests() {
  readSensor(0);
  readSensor(1);
  readSensor(2);
  readSensor(3);
  readSensor(4);
  readSensor(5);
  readSensor(6);
}
