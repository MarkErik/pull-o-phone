/*

This code uses the HX711 Library to read values from two HX711 Analog-to-Digital Converters conencted to Load Cells (I'm using the variables A and B to identify them)
By default the HX711 library takes 16 samples of data and averages them to get a more accurate value.
For a more responsive instrument, I set the number of samples to 1, so that we are sending as quick as the data comes in.
The values from the loadcells are then sent over the Serial Port to be read by SuperCollider, IFF there is a shape connected to the phone jack cables.
I detect the shapes by using a voltage divider circuit which is read by the Analog Input pins.

Author: Mark Altosaar
https://github.com/MarkErik/pull-o-phone

Incorporates Library:
Read from HX711 ADC (Load Cell): https://github.com/olkal/HX711_ADC

Voltage Divider Logic

If the jack has nothing connected to it (e.g. is unplugged), the voltage reading will be 5V,
thus will read a value of ~1023, and send over serial a value of -1

The following is the voltage readings assigned to the different shapes.

e - small box - resistor value of 2.36K (Red Dot) will give analog Pin reading of: 188
f - large square box - resistor value of 6.19K (Blue Dot) will give analog Pin reading of:  378
g - medium cylinder - resistor value of 14.6K (Green Dot) will give analog Pin reading of: ~601-602
h - sandpaper small square box - resistor value of 35K (Purple Dot) will give analog Pin reading of: 801

*/


#include <HX711_ADC.h>
#if defined(ESP8266) || defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

bool inRange(int reading, int target, int range) {
  return ((reading >= (target - range)) && (reading <= (target + range)));
}

//target readings for voltage divider
const int e_target = 188;
const int f_target = 378;
const int g_target = 601;
const int h_target = 801;
const int v_range = 8;  // +/-acceptable range for voltage reading

//Loadcell pins:
const int A_HX711_dout = 4;  //mcu > HX711 dout pin
const int A_HX711_sck = 5;   //mcu > HX711 sck pin
const int B_HX711_dout = 7;  //mcu > HX711 dout pin
const int B_HX711_sck = 6;   //mcu > HX711 sck pin

//HX711 constructor:
HX711_ADC A_LoadCell(A_HX711_dout, A_HX711_sck);
HX711_ADC B_LoadCell(B_HX711_dout, B_HX711_sck);

const int calVal_eepromAdress = 0;
unsigned long t = 0;

//Voltage reading pins
int A_sensorPin = A0;      // select the input pin for the voltage divider sensor for loadcell A
int A_voltageReading = 0;  // variable to store the value coming from the sensor

int B_sensorPin = A1;      // select the input pin for the voltage divider sensor for loadcell A
int B_voltageReading = 0;  // variable to store the value coming from the sensor

//values to send over the serial port
//set everything to 'not connected (-1)' at first
int e_value = -1;
int f_value = -1;
int g_value = -1;
int h_value = -1;

//SET TO TRUE TO PRINT ALL VALUES TO SERIAL PORT WITH PRINTLNS AND LABELS
//WON'T WORK WITH SUPERCOLLIDER IF SET TO TRUE
const bool testing = false;

void setup() {

  Serial.begin(115200);
  delay(6);

  pinMode(A_sensorPin, INPUT);
  pinMode(B_sensorPin, INPUT);

  A_LoadCell.begin();
  B_LoadCell.begin();

  float A_calibrationValue = -210.58;  // uncomment this if you want to set the calibration value in the sketch
  float B_calibrationValue = -203.18;  // uncomment this if you want to set the calibration value in the sketch

#if defined(ESP8266) || defined(ESP32)
  //EEPROM.begin(512); // uncomment this if you use ESP8266/ESP32 and want to fetch the calibration value from eeprom
#endif
  //EEPROM.get(calVal_eepromAdress, calibrationValue); // uncomment this if you want to fetch the calibration value from eeprom

  unsigned long stabilizingtime = 2000;  // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true;                  //set this to false if you don't want tare to be performed in the next step

  A_LoadCell.start(stabilizingtime, _tare);
  A_LoadCell.setSamplesInUse(1);
  if (A_LoadCell.getTareTimeoutFlag()) {
    if (testing) {
      Serial.println("A: Timeout, check MCU>HX711 wiring and pin designations, and restart");
    }
    while (1)
      ;
  } else {
    A_LoadCell.setCalFactor(A_calibrationValue);  // set calibration value (float)
    if (testing) {
      Serial.println("Loadcell A: Startup is complete");
    }
  }

  B_LoadCell.start(stabilizingtime, _tare);
  B_LoadCell.setSamplesInUse(1);
  if (B_LoadCell.getTareTimeoutFlag()) {
    if (testing) {
      Serial.println("B: Timeout, check MCU>HX711 wiring and pin designations, and restart");
    }
    while (1)
      ;
  } else {
    B_LoadCell.setCalFactor(B_calibrationValue);
    if (testing) {
      Serial.println("Loadcell B: Startup is complete");
    }
  }
}

void loop() {
  //variables to store whether new data is available from HX711 Load Cell ADC
  static boolean A_newDataReady = 0;
  static boolean B_newDataReady = 0;

  // check for new data/start next conversion:
  if (A_LoadCell.update()) A_newDataReady = true;
  if (B_LoadCell.update()) B_newDataReady = true;

  // read values from the loadcell sensors
  if (A_newDataReady || B_newDataReady) {
    float A_data = A_LoadCell.getData();
    float B_data = B_LoadCell.getData();
    if (A_data < 0) {
      A_data = 0;
    }
    if (B_data < 0) {
      B_data = 0;
    }

    //Read the voltage divider readings to detect if anything plugged into jacks
    A_voltageReading = analogRead(A_sensorPin);
    B_voltageReading = analogRead(B_sensorPin);

    if (inRange(A_voltageReading, 1023, v_range) && inRange(B_voltageReading, 1023, v_range)) {
      //nothing plugged into either jack, so no shapes active
      if (testing) {
        Serial.println("Nothing plugged into jacks.");
      }
      e_value = -1;
      f_value = -1;
      g_value = -1;
      h_value = -1;
    } else if (inRange(A_voltageReading, 1023, v_range)) {
      if (inRange(B_voltageReading, e_target, v_range)) {
        e_value = floor(B_data);
        f_value = -1;
        g_value = -1;
        h_value = -1;
      } else if (inRange(B_voltageReading, f_target, v_range)) {
        e_value = -1;
        f_value = floor(B_data);
        g_value = -1;
        h_value = -1;
      } else if (inRange(B_voltageReading, g_target, v_range)) {
        e_value = -1;
        f_value = -1;
        g_value = floor(B_data);
        h_value = -1;
      } else if (inRange(B_voltageReading, h_target, v_range)) {
        e_value = -1;
        f_value = -1;
        g_value = -1;
        h_value = floor(B_data);
      }
    }  //end else if nothing was plugged into jack A, and something was plugged into jack B
    else if (inRange(B_voltageReading, 1023, v_range)) {
      if (inRange(A_voltageReading, e_target, v_range)) {
        e_value = floor(A_data);
        f_value = -1;
        g_value = -1;
        h_value = -1;
      } else if (inRange(A_voltageReading, f_target, v_range)) {
        e_value = -1;
        f_value = floor(A_data);
        g_value = -1;
        h_value = -1;
      } else if (inRange(A_voltageReading, g_target, v_range)) {
        e_value = -1;
        f_value = -1;
        g_value = floor(A_data);
        h_value = -1;
      } else if (inRange(A_voltageReading, h_target, v_range)) {
        e_value = -1;
        f_value = -1;
        g_value = -1;
        h_value = floor(A_data);
      }
    }  //end else if nothing was plugged into jack B, and something was plugged into jack A
    else {
      if (inRange(A_voltageReading, e_target, v_range)) {
        e_value = floor(A_data);
      } else if (inRange(A_voltageReading, f_target, v_range)) {
        f_value = floor(A_data);
      } else if (inRange(A_voltageReading, g_target, v_range)) {
        g_value = floor(A_data);
      } else if (inRange(A_voltageReading, h_target, v_range)) {
        h_value = floor(A_data);
      }

      if (inRange(B_voltageReading, e_target, v_range)) {
        e_value = floor(B_data);
      } else if (inRange(B_voltageReading, f_target, v_range)) {
        f_value = floor(B_data);
      } else if (inRange(B_voltageReading, g_target, v_range)) {
        g_value = floor(B_data);
      } else if (inRange(B_voltageReading, h_target, v_range)) {
        h_value = floor(B_data);
      }
    }  //end the final case where there were two objects connected

    if (testing) {
      //Print all the data with labels for easier troubleshooting
      Serial.print("Loadcell A: ");
      Serial.println(A_data);
      Serial.print("Loadcell B:");
      Serial.println(B_data);

      Serial.print("Analog Voltage A:");
      Serial.println(A_voltageReading);
      Serial.print("Analog Voltage B:");
      Serial.println(B_voltageReading);

      Serial.print("E val:");
      Serial.println(e_value);
      Serial.print("F val:");
      Serial.println(f_value);
      Serial.print("G val:");
      Serial.println(g_value);
      Serial.print("H val:");
      Serial.println(h_value);
    } else {
      Serial.print(e_value);
      Serial.print("e");
      Serial.print(f_value);
      Serial.print("f");
      Serial.print(g_value);
      Serial.print("g");
      Serial.print(h_value);
      Serial.print("h");
    }

    //reset the booleans for whether the loadcell sensors are ready with data
    A_newDataReady = 0;
    B_newDataReady = 0;

  }  // end if new data was ready on the loadcells

  if (testing) {
    //Increase delay for printing values through the Arduino IDE Serial Monitor
    delay(120);
  } else {
    //send data over serial quickly for supercollider
    delay(1);
  }
}  //end main loop
