/* 
Currently receiving Heading PGN from mast compass on N2K network with Wind sensor
Using HTTP PUT to change mast compass settings
*/
#include <Arduino.h>
#include <ActisenseReader.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <N2kMessages.h>
#include <NMEA2000_esp32.h>
#include <NMEA0183Msg.h>
#include <NMEA0183Messages.h>
#include "NMEA0183Handlers.h"
#include "BoatData.h"
#include <ReactESP.h>
#include <Wire.h>
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>
#include <movingAvg.h>
#include "elapsedMillis.h"
#include <Arduino.h>
#include <N2kMessages.h>
#include <Adafruit_ADS1X15.h>
#include <WiFi.h>
#include "SPIFFS.h"
#include <Arduino_JSON.h>
#include <ESPmDNS.h>
#include "mcp2515.h"
#include "can.h"
#include "Async_ConfigOnDoubleReset_Multi.h"
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include <WebSerial.h>
#include <Adafruit_BNO08x.h>

#include "windparse.h"

// defs for robotshop CMPS14
//extern int CMPS14_ADDRESS;  // Address of CMPS14 shifted right one bit for arduino wire library
#define ANGLE_8  1          // Register to read 8bit angle from (starting...we read 5 bytes)
#define CAL_STATE 0x1E      // register to read calibration state

unsigned char high_byte, low_byte, angle8;
char pitch, roll;
unsigned int angle16, comp8, comp16;
#define VARIATION -15.2
static int variation;
float mastCompassDeg; 
float boatCompassDeg;
int boatCompassCalStatus;
float mastDelta;
// how is the compass oriented on the board relative to boat compass
// when mast is centered, mast compass+orientation == boat compass
int mastOrientation;
extern int mastFrequency;
extern bool compassOnToggle;
extern JSONVar readings;

extern bool compassOnToggle;

JSONVar board;

extern AsyncWebServer server;
extern AsyncEventSource events;
extern int mastAngle[];

float getCompass(int correction);
void logToAll(String message);
void logToAlln(String message);

// get heading from (local) compass
// called when we get a Heading PGN on the wind bus (which means mast compass transmitted)
// so frequency of update is going to depend on how often we get a Heading PGN from the mast
// also called as a Reaction in case we're not connected to mast compass
// TBD: make this object oriented and overload getCompass for either CMPS14 or BNO085
float getCMPS14(int correction) {
  //Serial.println("getCompass");
    Wire.beginTransmission(CMPS14_ADDRESS);  // starts communication with CMPS14
    Wire.write(ANGLE_8);                     // Sends the register we wish to start reading from
    Wire.endTransmission();
    // Request 5 bytes from the CMPS12
    // this will give us the 8 bit bearing, 
    // both bytes of the 16 bit bearing, pitch and roll
    Wire.requestFrom(CMPS14_ADDRESS, 5); 
    while((Wire.available() < 5)); // (this can hang?)
    angle8 = Wire.read();               // Read back the 5 bytes
    comp8 = map(angle8, 0, 255, 0, 359);
    comp8 = (comp8 + correction + 360) % 360;
    high_byte = Wire.read();
    low_byte = Wire.read();
    pitch = Wire.read();
    roll = Wire.read();
    // TBD set up PGN for attitude and transmit
    angle16 = high_byte;                 // Calculate 16 bit angle
    angle16 <<= 8;
    angle16 += low_byte;
    // doesn't this drop to an int?
    comp16 = ((angle16/10) + correction + 360) % 360;
//#define DEBUG
#ifdef DEBUG
    Serial.print("roll: ");               // Display roll data
    Serial.print(roll, DEC);
    
    Serial.print("    pitch: ");          // Display pitch data
    Serial.print(pitch, DEC);
    
    Serial.print("    angle full: ");     // Display 16 bit angle with decimal place
    Serial.print(angle16 / 10, DEC);
    Serial.print(".");
    Serial.print(angle16 % 10, DEC);

    Serial.print("    comp16: ");
    Serial.print(comp16, DEC);
    
    Serial.print("     comp8: ");        // Display 8bit angle
    Serial.println(comp8, DEC);
#endif
    Wire.beginTransmission(CMPS14_ADDRESS);  // starts communication with CMPS14
    Wire.write(CAL_STATE);                     // Sends the register we wish to start reading from
    Wire.endTransmission();
    Wire.requestFrom(CMPS14_ADDRESS, 5); 
    while((Wire.available() < 1)); // (this can hang)
    boatCompassCalStatus = Wire.read() & 0x3;
    //Serial.printf("calStatus: 0x%x\n", boatCompassCalStatus);
    return (float)comp16;
}

float calculateHeading(float r, float i, float j, float k, int correction);

// TBD make this a library shared between controller and mast compass
float getBNO085(int correction) {
  float accuracy, heading;
  // not checking timing here since it's controlled by ReactESP
  //unsigned long currentMillis = millis();
  //if (currentMillis - previousReading < BNOREADRATE) {
  //  logToAll("reading too soon" + String(currentMillis) + "-" + String(previousReading) + "\n");
  //  return -2.0; // minimum delay in case displayDelay is set too low
  //}
  //previousReading = currentMillis;

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 100))
      Serial.println("Could not enable rotation vector");
    else
      Serial.println("enabled abs rotation vector");
  }
  if (!bno08x.getSensorEvent(&sensorValue)) {
    return -3.0;
  }
  // printf("ID: %d\n", sensorValue.sensorId);
  /* Status of a sensor
   *   0 - Unreliable
   *   1 - Accuracy low
   *   2 - Accuracy medium
   *   3 - Accuracy high
   */
  boatCompassCalStatus = sensorValue.status;
  switch (sensorValue.sensorId) {
  case SH2_ROTATION_VECTOR:
    accuracy = sensorValue.un.rotationVector.accuracy;
    heading = calculateHeading(sensorValue.un.rotationVector.real, sensorValue.un.rotationVector.i, sensorValue.un.rotationVector.j, sensorValue.un.rotationVector.k, correction);      //logToAll("heading1: " + String(heading) + "  cal: " + calStatus + "\n");
    return heading;
    break;
  default:
    printf("ID: %d\n", sensorValue.sensorId);
    break;
  }
  return -4.0;
}

// Function to calculate tilt-compensated heading from a quaternion
float calculateHeading(float r, float i, float j, float k, int correction) {
  // Convert quaternion to rotation matrix
  float r11 = 1 - 2 * (j * j + k * k);
  // change r21 to =-2 if sensor is upside down
  float r21 = 2 * (i * j + r * k);
  float r31 = 2 * (i * k - r * j);
  float r32 = 2 * (j * k + r * i);
  float r33 = 1 - 2 * (i * i + j * j);

  // Calculate pitch (theta) and roll (phi)
  float theta = -asin(r31);
  float phi = atan2(r32, r33);
  // Calculate yaw (psi)
  float psi = atan2(r21, r11);
  float heading = (psi * 180 / M_PI) + correction;
  // correction may be positive or negative
  if (heading > 360) heading -= 360;
  if (heading < 0) heading += 360;
  return heading;
}

float getCompass(int correction) {
  if (!compassReady)
    return -1;
  if (compassType == CMPS14)
    return getCMPS14(correction);
  else if (compassType == BNO085)
    return getBNO085(correction);
  else
    return -2;
}

/*
https://www.arduino.cc/reference/en/language/functions/communication/wire/endtransmission/
endTransmission() returns:
0: success.
1: data too long to fit in transmit buffer.
2: received NACK on transmit of address.
3: received NACK on transmit of data.
4: other error.
5: timeout
*/
