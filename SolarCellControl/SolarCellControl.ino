/*
  Seth Eisner
  uses voltage divider and reads voltage across ground and analog pin 1
*/

#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>


Adafruit_BMP280 bmp;

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

const int chipSelect = 10;
const int voltPin = 1; // -> 37
const float MAXVOLTS = 71.53; //Must match voltage circuit resistors
float voltage = 0.0;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  pinMode(voltPin, INPUT);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("Initializing BMP\n");
  if (!bmp.begin()) {
    //Begin the sensor
    Serial.println("error");
    while (1);
  }
  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");
  File dataFile = SD.open("voltlog.csv", FILE_WRITE);
  dataFile.println("running time secs,voltage, temp C,elevation m");
  dataFile.close();
  Serial.println("Done Initializing BMP\n");

}

void loop() {
  // make a string for assembling the data to log:
  String dataString = "";
  float temperature = bmp.readTemperature();
  float elevation = bmp.readAltitude (1011);
  voltage = (float)analogRead(voltPin) * (MAXVOLTS / 1024.0);
  
  dataString = String(millis() / 1000) + ',';
  dataString += String(voltage, 1) + ',';
  dataString += String(temperature, 1) + ',';
  dataString += String(elevation, 1) ;

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.

  File dataFile = SD.open("voltlog.csv", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }

  //delay(1);
  delay(2000);
}
