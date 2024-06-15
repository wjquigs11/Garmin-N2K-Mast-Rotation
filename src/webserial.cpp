
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
#include "windparse.h"
#include <Arduino.h>
#include <N2kMessages.h>
#include <Adafruit_ADS1X15.h>
#include <WiFi.h>
//#include <AsyncTCP.h>
#include "SPIFFS.h"
#include <Arduino_JSON.h>
#include <ESPmDNS.h>
#include "mcp2515.h"
#include "can.h"
#include "Async_ConfigOnDoubleReset_Multi.h"
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include <WebSerial.h>

extern tBoatData *pBD;
extern tBoatData BoatData;
// TBD: I should add mag heading, mast heading etc (all the globals) to BoatData struct so I could have a single extern in each file

extern Stream *forward_stream;

extern tNMEA2000 *n2kMain;

// Honeywell sensor
extern movingAvg honeywellSensor;                // define the moving average object
extern int mastRotate, rotateout;
extern int PotValue, PotLo, PotHi;
extern Adafruit_ADS1015 ads;
extern int adsInit;

// defs for wifi
void initWebSocket();
//void notifyClients(String);
extern AsyncWebServer server;
extern bool serverStarted;
extern char *hostname;
extern int WebTimerDelay;
//extern AsyncWebSocket ws;
extern AsyncEventSource events;
extern JSONVar readings;
extern void setupWifi();
//extern String host;
extern void loopWifi();
void startWebServer();
String getSensorReadings();
void setupESPNOW();
bool sendMastControl();
extern DoubleResetDetector* drd;
void check_status();

// mast compass
int convertMagHeading(const tN2kMsg &N2kMsg); // magnetic heading of boat (from e.g. B&G compass)
float parseMastHeading(const tN2kMsg &N2kMsg);  // mast heading
float parseN2KHeading(const tN2kMsg &N2kMsg); // boat heading from external source
float getCompass(int correction);      // boat heading from internal ESP32 CMPS14
void httpInit(const char* serverName);
extern const char* serverName;
extern int mastOrientation;   // delta between mast compass and boat compass
extern float boatCompassDeg; // magnetic heading not corrected for variation
extern float mastCompassDeg;
extern int boatCompassCalStatus;
extern float mastDelta;

void mastHeading();
extern float mastAngle[2]; // array for both sensors
// 0 = honeywell
// 1 = compass

// robotshop CMPS14 (boat compass)
extern int CMPS14_ADDRESS;
extern bool cmps14_ready;

// Time after which we should reboot if we haven't received any CAN messages
#define MAX_RX_WAIT_TIME_MS 30000

extern bool displayOnToggle, compassOnToggle, honeywellOnToggle;


#ifdef ESPBERRY
void WindSpeed();
#else
void WindSpeed(const tN2kMsg &N2kMsg);
#endif
void BoatSpeed(const tN2kMsg &N2kMsg);


void logToAll(String s) {
  Serial.print(s);
  //consLog.print(s);
  if (serverStarted)
    WebSerial.print(s);
  s = String();
}

void i2cScan() {
  byte error, address;
  int nDevices = 0;
  logToAll("Scanning...");
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission(); 
    char buf[16];
    sprintf(buf, "%2X", address); // Formats value as uppercase hex
    if (error == 0) {
      logToAll("I2C device found at address 0x" + String(buf) + "\n");
      nDevices++;
    }
    else if (error == 4) {
      logToAll("error at address 0x" + String(buf) + "\n");
    }
  }
  if (nDevices == 0) {
    logToAll("No I2C devices found\n");
  }
  else {
    logToAll("done\n");
  }
}

//String wsCommands[] = {"?", "format", "restart"};

void WebSerialonMessage(uint8_t *data, size_t len) {
    Serial.printf("Received %lu bytes from WebSerial: ", len);
    Serial.write(data, len);
    Serial.println();
    WebSerial.println("Received Data...");
  String dataS = String((char*)data);
  // Split the String into an array of Strings using spaces as delimiters
  String words[10]; // Assuming a maximum of 10 words
  int wordCount = 0;
  int startIndex = 0;
  int endIndex = 0;
  while (endIndex != -1) {
    endIndex = dataS.indexOf(' ', startIndex);
    if (endIndex == -1) {
      words[wordCount++] = dataS.substring(startIndex);
    } else {
      words[wordCount++] = dataS.substring(startIndex, endIndex);
      startIndex = endIndex + 1;
    }
  }
  for (int i = 0; i < wordCount; i++) {    
    if (words[i].equals("?")) {
      WebSerial.println("format (spiffs)");
      WebSerial.println("restart");
      WebSerial.println("ls (spiffs)");
      WebSerial.println("scan (i2c)");
      WebSerial.println("status");
      WebSerial.println("readings (JSON)");
      WebSerial.println("mast");
    }
    if (words[i].equals("status")) {
      WebSerial.println("          AWS (in): " + String(WindSensor::windSpeedKnots) + "\n");
      WebSerial.println("          AWA (in): " + String(WindSensor::windAngleDegrees) + "\n");
      WebSerial.println("Sensor L/H/Current: " + String(PotLo)+ String(PotHi)+ String(PotValue) + "\n");
      WebSerial.println("      Sensor angle: " + String(mastRotate) + "\n");
      WebSerial.println("       Sorrect AWA: " + String(rotateout) + "\n");
      WebSerial.println("      Mast Compass: " + String(mastCompassDeg) + "\n");
      WebSerial.println("      Boat Compass: " + String(boatCompassDeg) + "\n");
      WebSerial.println("       Calibration: " + String(boatCompassCalStatus) + "\n");
      WebSerial.println("        Mast angle: " + String(mastDelta) + "\n");
    }
    if (words[i].equals("format")) {
      SPIFFS.format();
      WebSerial.println("SPIFFS formatted");
    }
    if (words[i].equals("restart")) {
      ESP.restart();
    }
    if (words[i].equals("ls")) {
      File root = SPIFFS.open("/");
      File file = root.openNextFile();
      while(file){
        WebSerial.println(file.name());
        file.close(); 
        file = root.openNextFile();
      }
      root.close();
      WebSerial.println("done");
    }
    if (words[i].equals("scan")) {
      i2cScan();
    }
    if (words[i].equals("readings")) {
      WebSerial.println(readings);
    }
    if (words[i].equals("mast")) {
      sendMastControl();
      //WebSerial.println(readings);
    }
  }
  for (int i=0; i<wordCount; i++) words[i] = String();
  dataS = String();
}
