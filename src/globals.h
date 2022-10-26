/*
 * Copyright (C) 2019-2021 OpenBikeSensor Contributors
 * Contact: https://openbikesensor.org
 *
 * This file is part of the OpenBikeSensor firmware.
 *
 * The OpenBikeSensor firmware is free software: you can
 * redistribute it and/or modify it under the terms of the GNU
 * Lesser General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * OpenBikeSensor firmware is distributed in the hope that
 * it will be useful, but WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with the OpenBikeSensor firmware.  If not,
 * see <http://www.gnu.org/licenses/>.
 */

#ifndef OBS_GLOBALS_H
#define OBS_GLOBALS_H

#include <Arduino.h>

// ----- choose your hardware here, uncomment only one -----
#define OBS_HARDWARE
//#define SIMPLE_OBS_HARDWARE


// ----- hardware configurations -----
#ifdef OBS_HARDWARE
const bool HAS_PIN_POWERED_DISPLAY = false;
const bool HAS_GPS_SWITCH = true;

const uint8_t PUSHBUTTON_PIN = 2;
const uint8_t GPS_POWER_PIN = 12;
const uint8_t BATTERY_VOLTAGE_PIN = 34;

const uint8_t SCK_SD_PIN = 18;
const uint8_t MISO_SD_PIN = 19;
const uint8_t MOSI_SD_PIN = 23;
const uint8_t CS_SD_PIN = 5;

const uint8_t ECHO1_PIN = 4;
const uint8_t TRIG1_PIN = 15 ;
const uint8_t ECHO2_PIN = 26;
const uint8_t TRIG2_PIN = 25;

const uint8_t RX_NEO6M_PIN = 17;
const uint8_t TX_NEO6M_PIN = 16;

const uint8_t SDA_DISPLAY_PIN = 21;
const uint8_t SCL_DISPLAY_PIN = 22;
const uint8_t VCC_DISPLAY_PIN = -1;     // can source ~20mA; enough to power the OLED 0.96" 
const uint8_t GND_DISPLAY_PIN = -1;     // can drain ~20mA; enough to power the OLED 0.96" 
                                        // gpio_set_drive_capability(GPIO_NUM_27, GPIO_DRIVE_CAP_3); 
                                        // ^this can be used in setup() to increase source/drain to 30-40mA 
const uint8_t displayAddress = 0x3c;

const double VBAT_R_KOHM = 150.0;       // high side R value (kOhm) of BatVoltage divider on PCB
const double GND_R_KOHM = 300.0;        // low side R value (kOhm) of BatVoltage divider on PCB
#else
#ifdef SIMPLE_OBS_HARDWARE
const bool HAS_PIN_POWERED_DISPLAY = true;
const bool HAS_GPS_SWITCH = false;

const uint8_t PUSHBUTTON_PIN = 21;
const uint8_t GPS_POWER_PIN = 33;
const uint8_t BATTERY_VOLTAGE_PIN = 35;

const uint8_t SCK_SD_PIN = 14;
const uint8_t MISO_SD_PIN = 2;
const uint8_t MOSI_SD_PIN = 15;
const uint8_t CS_SD_PIN = 13;

const uint8_t ECHO1_PIN = 22;
const uint8_t TRIG1_PIN = 19;
const uint8_t ECHO2_PIN = 23;
const uint8_t TRIG2_PIN = 18;

const uint8_t RX_NEO6M_PIN = 4;
const uint8_t TX_NEO6M_PIN = 5;

const uint8_t SDA_DISPLAY_PIN = 33;
const uint8_t SCL_DISPLAY_PIN = 25;
const uint8_t VCC_DISPLAY_PIN = 26;     // can source ~20mA; enough to power the OLED 0.96" 
const uint8_t GND_DISPLAY_PIN = 27;     // can drain ~20mA; enough to power the OLED 0.96" 
                                        // gpio_set_drive_capability(GPIO_NUM_27, GPIO_DRIVE_CAP_3); 
                                        // ^this can be used in setup() to increase source/drain to 30-40mA 
const uint8_t displayAddress = 0x3c;

const double VBAT_R_KOHM = 100.0;       // high side R value (kOhm) of BatVoltage divider on PCB
const double GND_R_KOHM = 100.0;        // low side R value (kOhm) of BatVoltage divider on PCB
#endif
#endif
// ----- end hardware configutration -----


// Forward declare classes to build (because there is a cyclic dependency between sensor.h and displays.h)
class SSD1306DisplayDevice;
class HCSR04SensorManager;

#include "utils/obsutils.h"
#include "config.h"
#include "displays.h"
#include "sensor.h"
#include "VoltageMeter.h"

// This file should contain declarations of all variables that will be used globally.
// The variables don't have to be set here, but need to be declared.

// Version
extern const char *OBSVersion;

extern int confirmedMeasurements;
extern int numButtonReleased;

extern Config config;


extern SSD1306DisplayDevice* displayTest;

extern HCSR04SensorManager* sensorManager;

class VoltageMeter;
extern VoltageMeter* voltageMeter;

class Gps;
extern Gps gps;

extern const uint32_t MAX_DURATION_MICRO_SEC;
extern const uint8_t LEFT_SENSOR_ID;
extern const uint8_t RIGHT_SENSOR_ID;
extern const uint16_t MAX_SENSOR_VALUE;

#endif
