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


SoftwareSerial bluetoothSerial(BLUETOOTH_TX, BLUETOOTH_RX);
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

bool test = true;
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
  ble.end(); // Flushes bluetooth connection if active
  if (!ble.begin(VERBOSE_MODE)){
	  error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  
  extensionSetting = 0;
  extensionLength = FULL_EXTENSION; // Used to determine how much the actuator extends

  delay(1000);
  
  Serial.println(F("Setup Pins")); setupPins(); // Setups the non-bluetooth pins
  
  Serial.println(F("Setup Sensors")); setupSensors(); // Setups the RGB Sensors // Loose pin somewhere in setupSensors()
  
// Process to setup bluetooth
  if(!test) {
    Serial.println(F("Setup Bluetooth..."));
    setupBluetooth();
    Serial.println(F("Finished Bluetooth Setup"));
  }
  
  //Setup Temperature Sensor and Heating Pad
  pinMode(BOARD_LED, OUTPUT);
  digitalWrite(BOARD_LED, LOW);
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
  
  // Sensor LED Pins
  pinMode(SENSOR_0_LED, OUTPUT);
  digitalWrite(SENSOR_0_LED, LOW);
  
  pinMode(SENSOR_1_LED, OUTPUT);
  digitalWrite(SENSOR_1_LED, LOW);
  
  pinMode(SENSOR_2_LED, OUTPUT);
  digitalWrite(SENSOR_2_LED, LOW);
  
  pinMode(SENSOR_3_LED, OUTPUT);
  digitalWrite(SENSOR_3_LED, LOW);
  
  pinMode(SENSOR_4_LED, OUTPUT);
  digitalWrite(SENSOR_4_LED, LOW);
  
  pinMode(SENSOR_5_LED, OUTPUT);
  digitalWrite(SENSOR_5_LED, LOW);
}

void setupBluetooth() {
  if (FACTORYRESET_ENABLE){
    Serial.println(F("Performing a factory reset: "));
    if (!ble.factoryReset()){
      error(F("Couldn't factory reset"));
    }
  }
  
  ble.echo(false); // Disables command echo from Bluefruit
  Serial.println("Requesting Bluefruit info:");
  ble.info(); /* Print Bluefruit information */
  ble.verbose(false); // disables debug info
  
  if (ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION))
  {
    // Change Mode LED Activity
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }
}

void setupSensors() {
  //Sets up RGB on MUX Channel 0
  tcaselect(0);
  tcs0 = Adafruit_TCS34725_modified(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
  tcs0.begin();

  //Sets up RGB on MUX Channel 1
  tcaselect(1);
  tcs1 = Adafruit_TCS34725_modified(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
  tcs1.begin();

  //Sets up RGB on MUX Channel 2
  tcaselect(2);
  tcs2 = Adafruit_TCS34725_modified(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
  tcs2.begin();

  //Sets up RGB on MUX Channel 3
  tcaselect(3);
  tcs3 = Adafruit_TCS34725_modified(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
  tcs3.begin();

  //Sets up RGB on MUX Channel 4
  tcaselect(4);
  tcs4 = Adafruit_TCS34725_modified(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
  tcs4.begin();

  //Sets up RGB on MUX Channel 5
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
  uint16_t clear, red, green, blue;
  int sensorLED;
  
  tcaselect(sensor);

  //digitalWrite(sensorLED, HIGH); // Turns sensorLED on
  digitalWrite(BOARD_LED, HIGH);
  
  switch(sensor){
    case 0:
      tcs0.enable();
      tcs0.getRawData(&red, &green, &blue, &clear);
      sensorLED = SENSOR_0_LED;
      break;
    case 1:
      tcs1.enable();
      tcs1.getRawData(&red, &green, &blue, &clear);
      sensorLED = SENSOR_1_LED;
      break;
    case 2:
      tcs2.enable();
      tcs2.getRawData(&red, &green, &blue, &clear);
      sensorLED = SENSOR_2_LED;
      break;
    case 3:
      tcs3.enable();
      tcs3.getRawData(&red, &green, &blue, &clear);
      sensorLED = SENSOR_3_LED;
      break;
    case 4:
      tcs4.enable();
      tcs4.getRawData(&red, &green, &blue, &clear);
      sensorLED = SENSOR_4_LED;
      break;
    case 5:
      tcs5.enable();
      tcs5.getRawData(&red, &green, &blue, &clear);
      sensorLED = SENSOR_5_LED;
      break;
  }

  float redf, bluef, greenf;
  redf = (float)red/(float)clear;
  greenf = (float)green/(float)clear;
  bluef = (float)blue/(float)clear;
  
  digitalWrite(BOARD_LED, LOW);
  //digitalWrite(sensorLED, LOW); // Turns sensorLED off


  // Don't think these are necessary
  //Serial.print(F("C:\t")); Serial.print(clear);
  //Serial.print(F("\tR:\t")); Serial.print(red);
  //Serial.print(F("\tG:\t")); Serial.print(green);
  //Serial.print(F("\tB:\t")); Serial.print(blue);
  //Serial.println();
  
  // To send sensor number as well for ordering
  //ble.print("AT+BLEUARTTX=");
  //ble.println(sensor);

  // Send values over bluetooth to app
  send('A', clear);
  send('R', red);
  send('G', green);
  send('B', blue);
  Serial.println("------------------------------------");
}

void send(char color, uint16_t val) {
  message[0] = color;31;
  dtostrf(val, sizeof(val), 0, &message[1]);
  Serial.print(F("Transmitting:\t")); Serial.println(message);
  ble.print("AT+BLEUARTTX=");
  ble.println(message);
  memset(&message[0], 0, sizeof(message));//deletes message
}

void drawFluid() {
  analogWrite(MOTOR, extensionLength);  
}

void returnFluid() {
  analogWrite(MOTOR, NO_EXTENSION);
}

void tcaselect(uint8_t i) {
  if (i > 7) return;
  //Serial.println("Pre begin transmission"); 
  Wire.beginTransmission(TCAADDR);
  //Serial.println("post begin transmission"); 
  Wire.write(1 << i);
  //Serial.println("Post write"); 
  Wire.endTransmission();  
  // Serial.println("Post end transmission");
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
