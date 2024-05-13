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

int mastOrientation, headingDeg;

//Flags to check whether new readings are available
boolean newMagOrient = false;
String sensorReadings;
const char* serverSSID = "ESPcompass";  // do this better later
const char* serverURL = "http://ESPcompass.local/readings";

extern JSONVar readings;
extern int mastAngle[];

void httpInit(const char* serverURL);
String httpGETRequest(const char* serverURL);

// called when we get a Mag Heading message from bus (i.e. from RPI, boat heading)
// checks mast heading (async) and prints difference (in degrees)
// RETURNS difference, if you want to xmit rudder angle add 50 (if mast range is -50..50)
int convertMagHeading(const tN2kMsg &N2kMsg) {
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
      headingDeg = heading * (180/M_PI);
      readings["heading"] = String(headingDeg);
      //Serial.print(headingDeg);
      // compare to heading reading from mast compass (via wifi)
      //Serial.print(" mast bearing: ");
      //Serial.print(mastOrientation);
      //Serial.print(" difference: ");
      int delta = mastOrientation - headingDeg;
      if (delta > 180) {
        delta -= 360;
      } else if (delta < -180) {
        delta += 360;
      }
      readings["mastDelta"] = String(delta); // TBD shift range to 0..100 for gauge
      mastAngle[1] = delta;
      //Serial.println(mastAngle[1]);
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
    sensorReadings = httpGETRequest(serverURL);
    if (sensorReadings.length()) {  // null string is 0 len
      //Serial.println(sensorReadings.c_str());
      JSONVar myObject = JSON.parse(sensorReadings);
      // JSON.typeof(jsonVar) can be used to get the type of the var
      if (JSON.typeof(myObject) == "undefined") {
        Serial.println("Parsing input failed!");
        mastOrientation = -1;
        return;
      }
      JSONVar keys = myObject.keys();    
      if (keys.length()) { // should be 1 key
        JSONVar value = myObject[keys[0]];
        //Serial.printf("mastheading got %d keys, value0 is %s\n", keys.length(), myObject[keys[0]].c_str());
        mastOrientation = atoi(value);
        readings["mastOrient"] = String(mastOrientation);
      }
    } else { // sensorReadings
      Serial.printf("GET failed: %d %s\n", sensorReadings.length(), serverURL);
      mastOrientation = -1;
    }
  } else {
    Serial.println(" WiFi Disconnected");
    mastOrientation = -1;
  }
}

WiFiClient client;
HTTPClient http;

void httpInit(const char* serverURL) { 
  // Your Domain name with URL path or IP address with path
  bool httpResult = http.begin(client, serverURL);
  Serial.printf("magHeading HTTP connect: %d\n", httpResult);
}

String httpGETRequest(const char* serverURL) {
  if (!http.connected())
      if (!http.begin(client, serverURL))
        Serial.println("http begin failed");
  //if (http.connected()) {
    int httpResponseCode = http.GET();
    //Serial.printf("HTTP response code %d\n", httpResponseCode);
    String payload = "{}";   
    if (httpResponseCode>0) {
      //Serial.printf("HTTP Response code: %d\n", httpResponseCode);
      payload = http.getString();
    } else {
      Serial.print("MagHeading HTTP GET Error code: ");
      Serial.println(httpResponseCode);
    }
    // Free resources
    //http.end();
    return payload;
  //} else { // not connected
  //  Serial.println("HTTP not connected");
  return String();
}

