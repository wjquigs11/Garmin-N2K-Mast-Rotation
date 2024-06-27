
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
//#include "mcp2515.h"
//#include "can.h"
#include "Async_ConfigOnDoubleReset_Multi.h"
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include <WebSerial.h>
#include <Adafruit_BNO08x.h>
#include "windparse.h"

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
//extern int CMPS14_ADDRESS;
//extern bool compassReady;

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
  Serial.println(s);
  //consLog.println(s);
  if (serverStarted)
    WebSerial.println(s);
  s = String();
}

void lsAPconn() {
  logToAll("AP connections");
  wifi_sta_list_t wifi_sta_list;
  tcpip_adapter_sta_list_t adapter_sta_list;
 
  memset(&wifi_sta_list, 0, sizeof(wifi_sta_list));
  memset(&adapter_sta_list, 0, sizeof(adapter_sta_list));
 
  esp_wifi_ap_get_sta_list(&wifi_sta_list);
  tcpip_adapter_get_sta_list(&wifi_sta_list, &adapter_sta_list);
 
  logToAll("stations: " + String(adapter_sta_list.num));
  for (int i = 0; i < adapter_sta_list.num; i++) {
    tcpip_adapter_sta_info_t station = adapter_sta_list.sta[i];
    logToAll("station nr " + String(i) + " MAC:");
    String printS;
    for(int i = 0; i< 6; i++){
      sprintf(prbuf, "%02X", station.mac[i]);
      printS += prbuf;
      if(i<5) printS += ".";
    }
    logToAll(printS);
    Serial.print("IP: ");  
    byte octet[4];
    octet[3] = station.ip.addr & 0xFF;
    octet[2] = (station.ip.addr >> 8) & 0xFF;
    octet[1] = (station.ip.addr >> 16) & 0xFF;
    octet[0] = (station.ip.addr >> 24) & 0xFF;
    printS = String(octet[3]) + "." + String(octet[2]) + "." + String(octet[1]) + "." + String(octet[0]);
    logToAll(printS);
    printS = String();
    }
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
      logToAll("I2C device found at address 0x" + String(buf));
      nDevices++;
    }
    else if (error == 4) {
      logToAll("error at address 0x" + String(buf));
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
    WebSerial.println(words[i]); 
    if (words[i].equals("?")) {
      WebSerial.println("format (spiffs)");
      WebSerial.println("restart");
      WebSerial.println("ls (spiffs)");
      WebSerial.println("scan (i2c)");
      WebSerial.println("status");
      WebSerial.println("readings (JSON)");
      WebSerial.println("mast");
      WebSerial.println("lsap (list access point connections)");
      WebSerial.println("toggle (check)");
      WebSerial.println("gps");
    }
    if (words[i].equals("status")) {
      WebSerial.println("          AWS (in): " + String(WindSensor::windSpeedKnots));
      WebSerial.println("          AWA (in): " + String(WindSensor::windAngleDegrees));
      WebSerial.println("Sensor L/H/Current: " + String(PotLo)+ String(PotHi)+ String(PotValue));
      WebSerial.println("      Sensor angle: " + String(mastRotate));
      WebSerial.println("       Sorrect AWA: " + String(rotateout));
      WebSerial.println("      Mast Compass: " + String(mastCompassDeg));
      WebSerial.println("      Boat Compass: " + String(boatCompassDeg));
      WebSerial.println("       Calibration: " + String(boatCompassCalStatus));
      WebSerial.println("        Mast angle: " + String(mastDelta));
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
      //sendMastControl();
      //WebSerial.println(readings);
    }
    if (words[i].equals("lsap")) {
      lsAPconn();
    }
    if (words[i].equals("toggle")) {
      WebSerial.println("Display: " + String(displayOnToggle));
      WebSerial.println("Compass: " + String(compassOnToggle));
      WebSerial.println("Honeywell: " + String(honeywellOnToggle));
    }
    if (words[i].equals("gps")) {
      WebSerial.println("Latitude: " + String(pBD->Latitude));
      WebSerial.println("Longitude: " + String(pBD->Longitude));
    }
  }
  for (int i=0; i<wordCount; i++) words[i] = String();
  dataS = String();
}
