#ifndef SETTINGS_H
#define SETTINGS_H

#include <Arduino.h>
#include <SoftwareSerial.h>

#include <Wire.h>
#include "Adafruit_TCS34725_modified.h"

//Pin definitions
#define BOARD_LED 11

#define SENSOR_0_LED 9
#define SENSOR_1_LED 8
#define SENSOR_2_LED 7
#define SENSOR_3_LED 6
#define SENSOR_4_LED 5
#define SENSOR_5_LED 4

#define MOTOR 10
#define NO_EXTENSION 0
#define ONE_QUARTER_EXTENSION 64
#define THREE_QUARTERS_EXTENSION 191
#define ONE_THIRD_EXTENSION 85
#define TWO_THIRDS_EXTENSION 170
#define HALF_EXTENSION 128
#define FULL_EXTENSION 255

#define LENGTH 25

#define BLUETOOTH_TX 3
#define BLUETOOTH_RX 2

#define NOT_LISTENING_TIME 1000

#endif // SETTINGS_H
