# 314159
Edited by Seth Eisner and Keenan Albee

Summer 2017

MegaHAB:

    High Altitude Balloon flight software adopted for use on an Arduino Mega.
    Combines GPS and APRS for use on the same Mega
    Uses: 
          Arduino Relay Shield 3.0 for cutting
          Adafruit BMP280 - Barometer
          Adafruit 9-DOF IMU Breakout - L3GD20H + LSM303
          Adafruit MicroSD card breakout board+
          ...And their related libraries
          
    Records Time data, latitude, longitude, GPS altitude, temperature, barometer altitude, XYZ accel magnetism and gyro data, and the state of each of the three cutters.
    Code makes use of 3 relay controlled cutters to make certain things happen at set altitudes
        1 and 2 are for science payloads and 3 is the emergency cutdown using relays 
        cutter 1 COM 4
        cutter 2 COM 3
        cutter 3 COM 2
    
Cell:

    Code to measure the voltage outputted by a solar cell (not needed for balloon flight)
