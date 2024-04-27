/*
Parse magnetic heading (typically will come from pypilot)
compare to magnetic heading sent via BLE from ESP32 with compass on mast
transmit on n2k as rudder angle for rudder #2
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

int magOrientation;

//Flags to check whether new readings are available
boolean newMagOrient = false;
String sensorReadings;
const char* serverSSID = "ESPcompass";  // do this better later
const char* serverName = "http://ESPcompass.local/readings";

void httpInit(const char* serverName);
String httpGETRequest(const char* serverName);

// called when we get a Mag Heading message from bus (i.e. from RPI, boat heading)
// checks mast heading (async) and prints difference (in degrees)
// RETURNS difference + 100 so we can xmit as rudder angle (range 0+)
int MagHeading(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  double heading;
  double deviation;
  double variation;
  tN2kHeadingReference headingRef;
  tN2kMsg correctN2kMsg; 

  //N2kMsg.Print(&Serial);
  if (ParseN2kPGN127250(N2kMsg, SID, heading, deviation, variation, headingRef) ) {
    #ifdef DEBUG2
    Serial.print("SID: "); Serial.print(SID);
    Serial.print(" heading: "); Serial.print(heading);
    Serial.print(" deviation: "); Serial.print(deviation);
    Serial.print(" variation: "); Serial.print(variation);
    Serial.print(" headingRef: "); Serial.print(headingRef);
    Serial.println();
    #endif
    if (headingRef == N2khr_magnetic && heading > 0) {  // need to check heading because it could be null
      //Serial.print("\tcompass heading: ");
      int headingDeg = heading * (180/M_PI);
      //Serial.print(headingDeg);
      // compare to heading reading from mast compass (via wifi)
      //Serial.print(" mast bearing: ");
      //Serial.print(magOrientation);
      //Serial.print(" difference: ");
      int delta = magOrientation - headingDeg;
      if (delta > 180) {
        delta -= 360;
      } else if (delta < -180) {
        delta += 360;
      }
      //Serial.println(delta);
      return delta;
    } else {
      Serial.println("no magnetic heading from main compass; doing nothing");
      return -1;
    }
  }
  return -1;
}

void mastHeading() {
  if(WiFi.status() == WL_CONNECTED) {       
    sensorReadings = httpGETRequest(serverName);
    JSONVar myObject = JSON.parse(sensorReadings);
    // JSON.typeof(jsonVar) can be used to get the type of the var
    if (JSON.typeof(myObject) == "undefined") {
      Serial.println("Parsing input failed!");
      magOrientation = -1;
    }
    JSONVar keys = myObject.keys();    
    if (keys.length()) { // should be 1 key
      JSONVar value = myObject[keys[0]];
      magOrientation = atoi(value);
    }
  } else {
    Serial.println(" WiFi Disconnected");
    magOrientation = -1;
  }
}

WiFiClient client;
HTTPClient http;

void httpInit(const char* serverName) { 
  // Your Domain name with URL path or IP address with path
  http.begin(client, serverName);
}

String httpGETRequest(const char* serverName) {
  if (!http.connected())
      http.begin(client, serverName);

  int httpResponseCode = http.GET();
  
  String payload = "{}"; 
  
  if (httpResponseCode>0) {
    //Serial.print("HTTP Response code: ");
    //Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  //http.end();

  return payload;
}

