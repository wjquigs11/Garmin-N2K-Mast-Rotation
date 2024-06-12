/*
Parse magnetic heading (typically will come from pypilot)
compare to magnetic heading sent from ESP32 with compass on mast
transmit on n2k as rudder angle
NOTE: NOT USING THIS CODE RIGHT NOW
Everything is in espnow.cpp since compass is the only ESPnow device
TBD: set global variation for correct compass display if we're not using internal compass 
*/
#include <Arduino.h>
#include <N2kMessages.h>
#include <NMEA2000_esp32.h>
#include <ReactESP.h>
#include <Wire.h>
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>
#include <vector>
#include <numeric>
#include <movingAvg.h>
#include <SPI.h>
#include "windparse.h"
#include <Arduino_JSON.h>
#include <WiFi.h>
#include <HTTPClient.h>

extern float boatCompassDeg;
extern float mastCompass;
extern int mastOrientation;

//Flags to check whether new readings are available
boolean newMagOrient = false;
String sensorReadings;
const char* serverSSID = "ESPcompass";  // do this better later
const char* serverURL = "http://ESPcompass.local/readings";

extern JSONVar readings;
extern int mastAngle[];

void httpInit(const char* serverURL);
String httpGETRequest(const char* serverURL);

// got heading PGN on main bus; only called if we do not have internal compass
float parseN2KHeading(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  double heading;
  double deviation;
  double variation;
  tN2kHeadingReference headingRef;

  //N2kMsg.Print(&Serial);
  if (ParseN2kPGN127250(N2kMsg, SID, heading, deviation, variation, headingRef)) {
    //#define DEBUG2
    #ifdef DEBUG2
    Serial.print("parseN2KHeading deviation: "); Serial.print(deviation);
    Serial.print(" variation: "); Serial.print(variation);
    Serial.print(" headingRef: "); Serial.print(headingRef);
    Serial.println();
    #endif
    if (heading >= 0) 
      return heading * (180/M_PI);
  } else Serial.printf("could not parse mast heading\n");
  return -1.0;
}

// got heading PGN on wind bus; set mast heading
// when we get a Wind PGN on bus, we will calculate heading difference and correct
// all we're doing here is setting mastHeadingDeg
float parseMastHeading(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  double heading;
  double deviation;
  double variation;
  tN2kHeadingReference headingRef;

  //N2kMsg.Print(&Serial);
  if (ParseN2kPGN127250(N2kMsg, SID, heading, deviation, variation, headingRef)) {
    //#define DEBUG2
    #ifdef DEBUG2
    Serial.print("parseMastHeading SID: "); Serial.print(SID);
    Serial.print(" heading: "); Serial.print(heading*180/M_PI);
    Serial.print(" deviation: "); Serial.print(deviation);
    Serial.print(" variation: "); Serial.print(variation);
    Serial.print(" headingRef: "); Serial.print(headingRef);
    Serial.println();
    #endif
    if (headingRef == N2khr_Unavailable && heading >= 0) {  
      //Serial.print("\mast heading: ");
      int mastHeadingDeg = heading * (180/M_PI);
      return mastHeadingDeg;
    } // else Serial.printf("not N2khr_Unavailable\n");
  } else Serial.printf("could not parse mast heading\n");
  return -1;
}

// called when we get a Mag Heading message from bus (i.e. from RPI, boat heading)
// checks mast heading (async) and prints difference (in degrees)
// RETURNS difference, if you want to xmit rudder angle add 50 (if mast range is -50..50)
// NOTE: this is ONLY for boat heading, not for mast heading
// will NOT be called if there is no external source of heading data like pypilot
// TBD: if there are 2 sources, decide which one to use
int convertMagHeading(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  double heading;
  double deviation;
  double variation;
  tN2kHeadingReference headingRef;
  tN2kMsg correctN2kMsg; 

  //N2kMsg.Print(&Serial);
  if (ParseN2kPGN127250(N2kMsg, SID, heading, deviation, variation, headingRef) ) {
    #define DEBUG2
    #ifdef DEBUG2
    Serial.print(" convertMagHeading SID: "); Serial.print(SID);
    Serial.print(" heading: "); Serial.print(heading*180/M_PI);
    Serial.print(" deviation: "); Serial.print(deviation);
    Serial.print(" variation: "); Serial.print(variation);
    Serial.print(" headingRef: "); Serial.print(headingRef);
    Serial.println();
    #endif
    if (headingRef == N2khr_magnetic && heading >= 0) {  // need to check heading because it could be null
      // TBD this duplicates work in wind-bus so change to shared function
      //Serial.print("\tcompass heading: ");
      boatCompassDeg = heading * (180/M_PI);
      //Serial.print(boatCompassDeg);
      // compare to heading reading from mast compass (via wifi)
      //Serial.print(" mast bearing: ");
      //Serial.print(mastOrientation);
      //Serial.print(" difference: ");
      int delta = mastOrientation - boatCompassDeg;
      if (delta > 180) {
        delta -= 360;
      } else if (delta < -180) {
        delta += 360;
      }
      mastAngle[1] = delta;
      //Serial.println(mastAngle[1]);
      // TBD: modify readings[] for web interface
      return delta;
    } else {
      Serial.println("no magnetic heading from main compass; doing nothing");
      return -1;
    }
  }
  return -1;
}
