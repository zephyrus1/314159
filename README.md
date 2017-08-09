# 314159 Keenan Albee and Seth Eisner

Adapted from [HABduino v4](https://github.com/HABduino/HABduino/tree/master/Software/habduino_v4)

Summer 2017

## MegaHAB:

**High Altitude Balloon flight software for use on an Arduino Mega.**

    Combines GPS and APRS for use on the same Mega
    Uses: 
          Arduino Relay Shield 3.0 for cutting
          Adafruit BMP280 - Barometer
          Adafruit 9-DOF IMU Breakout - L3GD20H + LSM303
          Adafruit MicroSD card breakout board+
          ...And their related libraries
          
    Varialble APRS_CALLSIGN needs to be changed to the user's radio (HAM) callsign 
    Variable seaLevelhPa is used for the barometer (BMP280)
    Records Time data, latitude, longitude, GPS altitude, temperature, barometer altitude, XYZ accel magnetism and gyro data, and the state of each of the three cutters.
    Code makes use of 3 relay controlled cutters to make certain things happen at set altitudes.
    Cutter 1 and 2 are for science payloads and 3 is the emergency cutdown using relays 
        
    CUT_#_ALT is the desired altituded the cutting happens at.
    CUT_#_LEN is the desired duration of the cut (time cutter is on) in seconds.
    CUT_#_TIMER is the redundant trigger for the cutter. The cutter will trigger, no matter the altitude, if the internal timer is greater than or equal to the set value. In seconds.
    CUT_#_PIN is the pin on the mega that the corresponds to the cutter specified by the #
    cut_#_progress is the state of the cutter specified by #
        0 = not started
        1 = in progress
        2 = done cutting
    cut_1_start_time doesnt need to be changed as it's set by the program. Ensures that the cutter runs.
| Cutter Number | Mega Pin | Shield Pin | COM (the green things) | 
| :---: | :---: | :---: | :---: |
| 1     | 22    | 4     | 4     |
| 2     | 23    | 5     | 3     |
| 3     | 24    | 6     | 2     |

    Users will only need to change the contents of the setup and loop functions to customize the software based on the sensors used and the data to be recorded
   
    
## SolarCellControl:

    Code to measure the voltage outputted by a solar cell (not needed for balloon flight)
    Arduino can only read up to 5 Volts, so the circuit to implement the code uses a voltage divider with a 1KΩ across the output voltage
        and 13KΩ as the other resistor. The Arduino measures the output voltage. [Adopted from this.](http://www.instructables.com/id/Arduino-Solar-Cell-Tester/)
