/*
 July 2017
 Keenan Albee
 Seth Eisner
 Modified from HABduino v4 software.

 This code allows simulataneous APRS and 434.485 MHz transmission using an Arduino Mega and HABduino shield. It is Arduino MEGA compatible, unlike the original software.
 The Arduino MEGA uses TIMER2 to perform PWM on pins 9 and 10, as opposed to pins 2,3 and 5 on the Uno. The timer regsters and ISR for TIMER2 have been updated to trigger pin 10 on overflow.
 HABduino shields will need pins 3 and 10 connected together for successful APRS use.

 This code reorganizes the HABduino v4 codebase into a more readable format. Useful modifiable variables are listed at the top, and relevant methods have been moved to separate files.

 An option to control a relay shield through an altitude trigger is also included, along with a sensor suite over I2C.

 ---

 HABDuino Tracker
 http://www.habduino.org
 (c) Anthony Stirk M0UPU

 March 2015 Version 4.0.0

 This is for the Version 4 Habduino Hardware.

 Credits :

 Interrupt Driven RTTY Code : Evolved from Rob Harrison's RTTY Code.
 Thanks to :  http://www.engblaze.com/microcontroller-tutorial-avr-and-arduino-timer-interrupts/
 http://gammon.com.au/power

 Suggestion to lock variables when making the telemetry string & Compare match register calculation from Phil Heron.
 APRS Code mainly by Phil Heron MI0VIM

 GPS Code modified from jonsowman and Joey flight computer CUSF
 https://github.com/cuspaceflight/joey-m/tree/master/firmware

 Thanks to :

 Phil Heron
 James Coxon
 Dave Akerman

 The UKHAS Community http://ukhas.org.uk

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 See <http://www.gnu.org/licenses/>.

 The hardware design & code for Habduino is released under a Creative Commons License 3.0 Attribution-ShareAlike License :
 See : http://creativecommons.org/licenses/by-sa/3.0/
 It is YOUR responsibility to ensure this kit is used safely please review the safety section.

 The latest code is available here : https://github.com/HABduino/HABduino
*/

//---Modifiable Variables---------------------------------------------------------------------------------------------------------

// TX and RX activation
#define APRS    // Comment to turn off APRS
//#define LOS     // Comment to turn off LOS (line of sight)
#define GPS     // Comment to turn off GPS

// TX variables
#define MTX2_FREQ 434.485 // LOS frequency, format 434.XXX
#define APRS_CALLSIGN "KC3JLF" // APRS callsign
char callsign[9] = "KC3JLF";  // LOS callsign, MAX 9 CHARACTERS
#define POWERSAVING      // Enables GPS powersaving mode
#define TXDELAY 0        // Delay between sentence TX's

// Cut variables
float seaLevelhPa = 1016.8; // pressure at sea level, hPa (yes, hectopascals)
float CUT_ALT_1 = 24615; // cut altitude, m -- 80,000 feet
float CUT_ALT_2 = 27692; //90,000 for reflector
const int CUT_TIME = 10000; // cut duration, msec

// Advanced TX variables (not recommeneded for modification)
#define ASCII 7          // ASCII 7 or 8
#define STOPBITS 2       // Either 1 or 2
#define RTTY_BAUD 100     // RTTY Baud rate (Recommended = 50)
#define BAUD_RATE      (1200)
#define TABLE_SIZE     (512)
#define PREAMBLE_BYTES (50)
#define REST_BYTES     (5)
//#define MTX2_SHIFT 425         // Uncomment to use
//#define MTX2_OFFSET 0          // Uncomment to use: 0-100 Slightly adjusts the frequency by increasing the PWM

//---Private Variables---------------------------------------------------------------------------------------------------------

//Nichrome cutters
int CUT1_PIN = 22;
//fix this pin number
int CUT2_PIN = 999;
//end of fix this
int cut1_progress = 0; //0 = not started, 1 = in progress, 2 = done
int cut2_progress = 0; //0 = not started, 1 = in progress, 2 = done
long start_1_time = 0;
long start_2_time = 0;

//Sensors
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#include <util/crc16.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SoftwareSerial.h>
#include "ax25modem.h"
static const uint8_t PROGMEM _sine_table[] = {
  #include "sine_table.h"
};

#include <Adafruit_Sensor.h>
#include <Adafruit_9DOF.h>

#include <Adafruit_L3GD20_U.h> //UNIFIED GYRO
#include <Adafruit_L3GD20.h>

#include <Adafruit_LSM303_U.h> //ACCELERATION & MAGNETISM
#include <Adafruit_LSM303.h>

#include <Adafruit_BMP280.h> //PRESSURE & TEMPERATURE

File logFile;
String data = "";

float latitude_  = -999.0;
float longitude_ = -999.0;
long  altitude_  = -999;

Adafruit_L3GD20_Unified         gyro  = Adafruit_L3GD20_Unified(20); //gyro
Adafruit_BMP280                 bmp;                  //temp and pres
Adafruit_LSM303                 lsm;                  //accel and mag
Adafruit_9DOF                   dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified   accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified     mag   = Adafruit_LSM303_Mag_Unified(30302);
const int chipSelect = 53;

#define BATTERY_ADC A0
#define GPS_ON 2
#define MTX2_TXD 4
#define ONE_WIRE_BUS 5
#define HX1_ENABLE 6
#define MTX2_ENABLE 7
#define LED_OK 8
#define LED_WARN 9

//===Modified for Mega================================================
#define HX1_TXD 10
//===Modified for Mega================================================

#define ONE_SECOND F_CPU / 1024 / 16

#define PLAYBACK_RATE    (F_CPU / 256)
#define SAMPLES_PER_BAUD (PLAYBACK_RATE / BAUD_RATE)
#define PHASE_DELTA_1200 (((TABLE_SIZE * 1200L) << 7) / PLAYBACK_RATE)
#define PHASE_DELTA_2200 (((TABLE_SIZE * 2200L) << 7) / PLAYBACK_RATE)
#define PHASE_DELTA_XOR  (PHASE_DELTA_1200 ^ PHASE_DELTA_2200)

SoftwareSerial MTX2_EN(12, MTX2_ENABLE); // RX, TX
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

volatile int txstatus=1;
volatile int txstringlength=0;
volatile char txc;
volatile int txi;
volatile int txj;
volatile int count=1;
volatile boolean lockvariables = 0;
volatile static uint8_t *_txbuf = 0;
volatile static uint8_t  _txlen = 0;

int errorstatus=0;
/* Error Status Bit Level Field :
 Bit 0 = GPS Error Condition Noted Switch to Max Performance Mode
 Bit 1 = GPS Error Condition Noted Cold Boot GPS
 Bit 2 = DS18B20 temp sensor status 0 = OK 1 = Fault
 Bit 3 = Current Dynamic Model 0 = Flight 1 = Pedestrian
 Bit 4 = PSM Status 0 = PSM On 1 = PSM Off
 Bit 5 = Lock 0 = GPS Locked 1= Not Locked

 So error 8 means the everything is fine just the GPS is in pedestrian mode.
 Below 1000 meters the code puts the GPS in the more accurate pedestrian mode.
 Above 1000 meters it switches to dynamic model 6 i.e flight mode and turns the LED's off for additional power saving.
 So as an example error code 40 = 101000 means GPS not locked and in pedestrian mode.
 */

char txstring[100];
uint8_t buf[60];

uint8_t lock =0, sats = 0, hour = 0, minute = 0, second = 0;
uint8_t oldhour = 0, oldminute = 0, oldsecond = 0;
int GPSerror = 0,navmode = 0,psm_status = 0,lat_int=0,lon_int=0,tempsensors,temperature1=0,temperature2=0;
int32_t lat = 0, lon = 0, alt = 0, maxalt = 0, lat_dec = 0, lon_dec =0 ,tslf=0;
unsigned long currentMillis;
long previousMillis = 0;
int batteryadc_v,battvaverage=0,aprstxstatus=0;
int32_t battvsmooth[5] ;
int aprs_tx_status = 0, aprs_attempts = 0;
unsigned long startTime;
char comment[3]={
  ' ', ' ', '\0'};

//---Setup---------------------------------------------------------------------------------------------------------

void setup()  {

  //Pin setup, NOTE: blinkled is a debugging helper to show completion of code
  pinMode(MTX2_TXD, OUTPUT);
  pinMode(LED_WARN, OUTPUT);
  pinMode(HX1_ENABLE, OUTPUT);
  pinMode(LED_OK,OUTPUT);
  pinMode(MTX2_ENABLE, OUTPUT);
  pinMode(GPS_ON, OUTPUT);
  pinMode(BATTERY_ADC, INPUT);
  pinMode(CUT1_PIN, OUTPUT); //Cutter
  pinMode(3, INPUT); //junk pin
  blinkled(2);

  Serial.begin(9600);
  #ifdef LOS
    setMTX2Frequency();
    digitalWrite(MTX2_ENABLE,HIGH);
    blinkled(2);
  #endif

  #ifdef GPS
    //GPS activate
    digitalWrite(GPS_ON,HIGH);
    blinkled(2);

    //GPS reset
    resetGPS();
    blinkled(2);
  #endif GPS

  //APRS setup
  #ifndef APRS
    TCCR2B = TCCR2B & 0b11111000 | 1; // Sets fast PWM on pin 11
  #endif
  #ifdef APRS
    ax25_init();
    blinkled(2);
  #endif

  //GPS setup
  #ifdef GPS
    setupGPS();
    blinkled(2);
  #endif GPS

  initialise_interrupt();

  //I2C setup
  sensors.begin();
  tempsensors=sensors.getDeviceCount();

  /*
  Sensor setup
  */

  //SD card setup
  Serial.println("Initializing SD card...");
  if(chipSelect){
    if(!SD.begin(chipSelect)){
      Serial.println("ERROR: SD CARD FAILED");
      while(1);
    }
  }
  logFile = SD.open("log.csv", FILE_WRITE);
  logFile.println("Beginning new log at "+String(hour)+":"+String(minute)+":"+String(second));
  logFile.close();
  Serial.println("SD card initialized.");

  //Gyro setup
  gyro.enableAutoRange(true);
  Serial.println("Initializing L3GD20...");
  if (!gyro.begin()) {
      Serial.println("ERROR: NO L3GD20 DETECTED");
      while(1);
  }
  Serial.println("L3GD20 initialized.");

  //Temp+altimeter setup
  Serial.println("Initializing BMP280...");
  if(!bmp.begin()){
      Serial.println("ERROR: NO BMP280 DETECTED");
      while(1);
  }
  Serial.println("BMP280 initialized.");

  //Accelerometer setup
  Serial.println("Initializing LSM303...");
  if(!lsm.begin()){
    Serial.println("ERROR: NO LSM303 DETECTED - 0");
    while(1);
  }
  if(!accel.begin()){
    Serial.println("ERROR: NO LSM303 DETECTED - 1");
    while(1);
  }
  if(!mag.begin()){
    Serial.println("ERROR: NO LSM303 DETECTED - 2");
    while(1);
  }
  Serial.println("LSM303 initialized.");

  //Completion of sensor setup
  Serial.println("Sensors initialized.");
  blinkled(2);
}

//---Loop---------------------------------------------------------------------------------------------------------

void loop() {
  oldhour=hour;
  oldminute=minute;
  oldsecond=second;

  //GPS update
  #ifdef GPS
    gps_check_nav();
  #endif

  //APRS send, NOTE: needs sufficient satellite count
  #ifdef APRS
    if(sats>=4){ //MODIFIED
      if (aprs_tx_status==0)
      {
        startTime=millis();
        aprs_tx_status=1;
      }
      if(millis() - startTime > (APRS_TX_INTERVAL*1000)) {
        aprs_tx_status=0;
        send_APRS();
        Serial.println("Sent APRS.");
        aprs_attempts++;
      }
    }
  #endif

  //GPS checks
  #ifdef GPS
    if(lock!=3) { // Blink LED to indicate no lock
      errorstatus |=(1 << 5);  // Set bit 5 (Lock 0 = GPS Locked 1= Not Locked)
    }
    else {
      errorstatus &= ~(1 << 5); // Unset bit 5 (Lock 0 = GPS Locked 1= Not Locked)
    }
    checkDynamicModel();

    //Powersaving mode
    #ifdef POWERSAVING
      if((lock==3) && (psm_status==0) && (sats>=5) &&((errorstatus & (1 << 0))==0)&&((errorstatus & (1 << 1))==0)) { // Check we aren't in an error condition
        setGPS_PowerSaveMode();
        wait(1000);
        psm_status=1;
        errorstatus &= ~(1 << 4); // Set Bit 4 Indicating PSM is on
      }
    #endif

    if(!lockvariables) {
      prepare_data();
      if(alt>maxalt && sats >= 4)
      {
        maxalt=alt;
      }
    }
    if((oldhour==hour&&oldminute==minute&&oldsecond==second)||sats<=4) {
      tslf++;
    }
    else {
      tslf=0;
      errorstatus &= ~(1 << 0); // Unset bit 0 (Clear GPS Error Condition Noted Switch to Max Performance Mode)
      errorstatus &= ~(1 << 1); // Unset bit 1 (Clear GPS Error Condition Noted Cold Boot GPS)
    }
    if((tslf>10 && ((errorstatus & (1 << 0))==0)&&((errorstatus & (1 << 1))==0))) {
      setupGPS();
      wait(125);
      setGps_MaxPerformanceMode();
      wait(125);
      errorstatus |=(1 << 0); // Set Bit 1 (GPS Error Condition Noted Switch to Max Performance Mode)
      psm_status=0;
      errorstatus |=(1 << 4); // Set Bit 4 (Indicate PSM is disabled)
    }
    if(tslf>100 && ((errorstatus & (1 << 0))==1)&&((errorstatus & (1 << 1))==0)) {
      errorstatus |=(1 << 0); // Unset Bit 0 we've already tried that didn't work
      errorstatus |=(1 << 1); // Set bit 1 indicating we are cold booting the GPS
      Serial.flush();
      resetGPS();
      wait(125);
      setupGPS();
    }
  #endif GPS

  /*
  Data gathering for CSV
  */

  data = "";

  sensors_event_t   event;
  sensors_event_t   accel_event;
  sensors_event_t   mag_event;
  sensors_vec_t     orientation;

  gyro.getEvent(&event);
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);

  lsm.read();

  data = data + String(hour)                    +':';
  data = data + String(minute)                  +':';
  data = data + String(second)                  +',';
  data = data + String(millis())                + ',';
  if(latitude_ >= -90.0 && latitude_ <= 90.0){
    data = data + String(latitude_, 6)         + ',';
  }
  else{
    data = data + "N/A,";
  }
  if(longitude_ >= -180.0 && longitude_ <= 180.0){
    data = data + String(longitude_, 6)        + ',';
  }
  else{
    data = data + "N/A,";
  }
  if(altitude_ >= 0){
    data = data + String(altitude_)            + ',';
  }
  else{
    data = data + "N/A,";
  }
  data = data + String(sats)                   + ',';

  //Sensor data
  data = data + String(bmp.readTemperature())         + ',';
  data = data + String(bmp.readPressure()/100)            + ',';
  data = data + bmp.readAltitude(seaLevelhPa) + ',';
  data = data + String((int)lsm.accelData.x)  + ',';
  data = data + String((int)lsm.accelData.y)  + ',';
  data = data + String((int)lsm.accelData.z)  + ',';
  data = data + String((int)lsm.magData.x)    + ',';
  data = data + String((int)lsm.magData.y)    + ',';
  data = data + String((int)lsm.magData.z)    + ',';
  data = data + event.gyro.z                  + ',';
  data = data + event.gyro.y                  + ',';
  data = data + event.gyro.z                  + ',';

  if(dof.accelGetOrientation(&accel_event, &orientation)){
    data = data + String(orientation.roll)    + ',';
    data = data + String(orientation.pitch)   + ',';
  }
  else{
    data = data + "N/A,N/A,";
  }

  if(dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation)){
    data = data + String(orientation.heading);
  }
  else{
    data = data + "N/A";
  }

  Serial.println(String(data)); //data = hour:minute:second,millis,lat(deg),lon(deg),GPS alt(m),external sensor temp(C),pressure(hPa),barometer alt (m),x_accel,y_accel,z_accel,x_mag,y_mag,z_mag,z_rate,y_rate,x_rate,roll,pitch,yaw

  //Write to SD card
  if(SD.exists("log.csv")) {
    Serial.println("Writing data to log file...");
    logFile = SD.open("log.csv", FILE_WRITE);
    logFile.println(data);
    logFile.close();
  }
  else{
    Serial.println("ERROR: UNABLE TO OPEN LOG FILE");
  }

  //Nichrome cutter code
  alt = bmp.readAltitude(seaLevelhPa);
  //cut 1
  if (alt >= CUT_ALT_1 && cut1_progress == 0){
    Serial.println("Cutting 1 begun...");
    cut1_progress = 1; // in progress
    start_1_time = millis();
    digitalWrite(CUT1_PIN, HIGH);
  }
  if (cut1_progress == 1 && (millis()-start_1_time) >= CUT_TIME) {
    cut1_progress = 2; // complete
    digitalWrite(CUT1_PIN, LOW);
    Serial.println("...cutting 1 complete.");
  }
  //cut 2
  if(alt >= CUT_ALT_2 && cut2_progress == 0){
      Serial.println("Cutting 2 begun...");
      cut2_progress = 1; // in progress
      start_2_time = millis();
      digitalWrite(CUT2_PIN, HIGH);
  }
  if (cut2_progress == 1 && (millis()-start_2_time) >= CUT_TIME) {
      cut2_progress = 2; // complete
      digitalWrite(CUT2_PIN, LOW);
      Serial.println("...cutting 2 complete.");
  }

  /*
  //Add more cutdowns here
  */

  //Serial.println(millis()-start__time);

  delay(250); // get rid of this?
}

//---Helper code----------------------------------------------------------------------------------------------

void blinkled(int blinks) {
  for(int blinkledx = 0; blinkledx < blinks; blinkledx++) {
    digitalWrite(LED_WARN,HIGH);
    wait(100);
    digitalWrite(LED_WARN,LOW);
    wait(100);
  }
}

void wait(unsigned long delaytime) {
  // Arduino Delay doesn't get CPU Speeds below 8Mhz
  unsigned long _delaytime=millis();
  while((_delaytime+delaytime)>=millis()){
  }
}

void prepare_data() {
  //Get data from HABduino sensors for telemetry if APRS is not currently transmitting
  if(aprstxstatus==0)
  {
    sensors.requestTemperatures();
    temperature1=sensors.getTempCByIndex(0);
    if(tempsensors==2)
    {
      temperature2=sensors.getTempCByIndex(1);
    }
    if(temperature1==-127)
    {
      errorstatus |=(1 << 2); // Set bit 2 indicating the temp sensor is faulty
    }
    else
    {
      errorstatus &= ~(1 << 2); // Unset bit 2 indicating temp sensor is ok.
    }
  }
  gps_check_lock();
  gps_get_position();
  gps_get_time();
  batteryadc_v=analogRead(BATTERY_ADC)*4.8;
  battvsmooth[4] = battvsmooth[3];
  battvsmooth[3] = battvsmooth[2];
  battvsmooth[2] = battvsmooth[1];
  battvsmooth[1] = battvsmooth[0];
  battvsmooth[0] = batteryadc_v;
  battvaverage = (battvsmooth[0]+battvsmooth[1]+ battvsmooth[2]+battvsmooth[3]+battvsmooth[4])/5;
}
