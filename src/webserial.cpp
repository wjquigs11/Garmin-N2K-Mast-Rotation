
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
extern AsyncEventSource events;
extern JSONVar readings;
extern void setupWifi();
//extern String host;
extern void loopWifi();
void startWebServer();

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
#ifdef PICOMPASS
extern float boatCompassPi;
extern int piCompCount;
#endif
extern int boatCalStatus;
#ifdef CMPS14
extern byte calibrationStatus[];
#endif
extern float mastDelta;

extern int num_n2k_messages;
extern int num_wind_messages;

void mastHeading();
extern int mastAngle[2]; // array for both sensors
// 0 = honeywell
// 1 = compass

extern bool displayOnToggle, compassOnToggle, honeywellOnToggle;
extern bool teleplot;

#ifdef RS485CAN
void WindSpeed();
#else
void WindSpeed(const tN2kMsg &N2kMsg);
#endif
void BoatSpeed(const tN2kMsg &N2kMsg);

#ifdef ESPNOW
void espNowBroadcast();
extern bool foundPeer;
extern int rxCount;
#endif

extern Adafruit_BNO08x bno08x;

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
    //Serial.print("IP: ");  
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

String commandList[] = {"?", "format", "restart", "ls", "scan", "status", "readings", "mast", "lsap", "toggle", "gps", "webserver", "compass", "windrx", "espnow", "teleplot"};
#define ASIZE(arr) (sizeof(arr) / sizeof(arr[0]))
String words[10]; // Assuming a maximum of 10 words

void WebSerialonMessage(uint8_t *data, size_t len) {
  //Serial.printf("Received %lu bytes from WebSerial: ", len);
  Serial.write(data, len);
  //Serial.println();
  ////Serial.printf("commandList size is: %d\n", ASIZE(commandList));
  WebSerial.println("Received Data...");
  String dataS = String((char*)data);
  // Split the String into an array of Strings using spaces as delimiters
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
    int j;
    if (words[i].equals("?")) {
      for (j = 1; j < ASIZE(commandList); j++) {
        WebSerial.println(String(j) + ":" + commandList[j]);
      }
    }
    if (words[i].toInt() > 0)
      for (j = 1; j < ASIZE(commandList); j++) {
        //WebSerial.println("j: " + String(j) + " " + commandList[j]);
        if (words[i].toInt() == j) {
          //Serial.printf("match %d %s %d %s\n", i, words[i].c_str(), j, commandList[j].c_str());
          words[i] = commandList[j];
        }
      }
    if (words[i].equals("status")) {
      WebSerial.println("             uptime: " + String(millis() / 1000));
      WebSerial.println("           AWS (in): " + String(WindSensor::windSpeedKnots));
      WebSerial.println("           AWA (in): " + String(WindSensor::windAngleDegrees));
      WebSerial.println(" Sensor L/H/Current: " + String(PotLo) + "/" + String(PotHi) + "/" + String(PotValue));
      WebSerial.println("       Sensor angle: " + String(mastRotate));
      WebSerial.println("        Correct AWA: " + String(rotateout));
      WebSerial.println("       Mast Compass: " + String(mastCompassDeg));
      WebSerial.println("         Mast angle: " + String(mastDelta));
      WebSerial.println("           n2k main: " + String(num_n2k_messages));
      WebSerial.println("           n2k wind: " + String(num_wind_messages));
    }
    if (words[i].equals("compass")) {
      WebSerial.println("           Boat Compass: " + String(boatCompassDeg));
      WebSerial.println("       Calibration(0-3): " + String(boatCalStatus));
#ifdef PICOMPASS
      WebSerial.println("        Boat Pi Compass: " + String(boatCompassPi));
      WebSerial.println("                 piComp: " + String(piCompCount));
#endif
#ifdef CMPS14
      WebSerial.printf("            [Calibration]: ");
      String CV = String((uint16_t)((calibrationStatus[0] << 8) | calibrationStatus[1]));
      WebSerial.println("mag: " + String(calibrationStatus[0]) + String(calibrationStatus[1]) + " " + CV);
      CV = String((calibrationStatus[2] << 8) | calibrationStatus[3]);
      WebSerial.println("acc: " + String(calibrationStatus[2]) + String(calibrationStatus[3]) + " " + CV);
      // gyro cal is "currently broken"
      CV = String((calibrationStatus[6] << 8) | calibrationStatus[7]);
      WebSerial.println("sys: " + String(calibrationStatus[6]) + String(calibrationStatus[7]) + " " + CV);
      WebSerial.println();
      CV = String();
#endif
    }
    if (words[i].equals("format")) {
      SPIFFS.format();
      WebSerial.println("SPIFFS formatted");
    }
    if (words[i].equals("restart")) {
      WebSerial.println("Restarting...");
      ESP.restart();
    }
    if (words[i].equals("ls")) {
      File root = SPIFFS.open("/");
      File file = root.openNextFile();
      while (file) {
        WebSerial.println(file.name());
        file.close(); 
        file = root.openNextFile();
      }
      root.close();
      //WebSerial.println("done");
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
    if (words[i].equals("gps") && pBD) {
      //Serial.printf("gps coords: %2.2d %2.2d\n", pBD->Latitude, pBD->Longitude);
      WebSerial.println("Latitude: " + String(pBD->Latitude));
      WebSerial.println("Longitude: " + String(pBD->Longitude));
    }
    if (words[i].equals("webserver")) {
      WebSerial.print("local IP: ");
      WebSerial.println(WiFi.localIP());
      WebSerial.print("AP IP address: ");
      WebSerial.println(WiFi.softAPIP());
        int clientCount = WiFi.softAPgetStationNum();
        if (clientCount > 0) {
          WebSerial.print("Clients connected: ");
          WebSerial.println(clientCount);
        } else {
          WebSerial.println("No clients connected");
        }

    }
    if (words[i].equals("windrx")) {
      WebSerial.println("last wind time: " + String(time_since_last_wind_rx) + " avg wind time: " + String(avg_time_since_last_wind) + " ms");
      if (time_since_last_wind_rx > 0.0)
        WebSerial.println(String(1000.0/avg_time_since_last_wind) + " Hz (confirm timing 1000?)");
    }    
    if (words[i].equals("espnow")) {
      WebSerial.println("espnow peer? " + String(foundPeer));
      WebSerial.println("espnow rx count: " + String(rxCount));
      WebSerial.println("sending broadcast...");
      espNowBroadcast();
    }
    if (words[i].equals("teleplot")) {
      teleplot = !teleplot;
      logToAll("teleplot: " + teleplot?"on":"off");
      return;
    }
   }
  for (int i=0; i<wordCount; i++) words[i] = String();
  dataS = String();
}
