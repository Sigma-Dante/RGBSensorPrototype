#ifndef SETTINGS_H
#define SETTINGS_H

#include <Arduino.h>
#include <SoftwareSerial.h>

#include <Wire.h>
#include "Adafruit_TCS34725_modified.h"

//Pin definitions
#define BOARD_LED 11

#define MOTOR 10
#define NO_EXTENSION 0
#define ONE_QUARTER_EXTENSION 64
#define THREE_QUARTERS_EXTENSION 191
#define ONE_THIRD_EXTENSION 85
#define TWO_THIRDS_EXTENSION 170
#define HALF_EXTENSION 128
#define FULL_EXTENSION 255

#define LENGTH 25

#define NOT_LISTENING_TIME 1000

#endif // SETTINGS_H
