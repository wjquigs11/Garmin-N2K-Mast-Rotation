
#include "windparse.h"
#include "BoatData.h"

// object-oriented classes
#include "BNO085Compass.h"
#include "logto.h"

// TBD: I should add mag heading, mast heading etc (all the globals) to BoatData struct so I could have a single extern in each file

extern Stream *forward_stream;

extern tNMEA2000 *n2kMain;

#ifdef HONEY
// Honeywell sensor
extern movingAvg honeywellSensor;                // define the moving average object
extern int mastRotate, rotateout;
extern int PotValue, PotLo, PotHi;
extern Adafruit_ADS1015 ads;
extern int adsInit;
#endif

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
void writeWiFi(int priority, String ssidNew, String passwdNew);

// mast compass
int convertMagHeading(const tN2kMsg &N2kMsg); // magnetic heading of boat (from e.g. B&G compass)
float parseMastHeading(const tN2kMsg &N2kMsg);  // mast heading
float parseN2KHeading(const tN2kMsg &N2kMsg); // boat heading from external source
float getCompass(int correction);      // boat heading from internal ESP32 CMPS14
void httpInit(const char* serverName);
extern const char* serverName;
extern int mastOrientation;   // delta between mast compass and boat compass
extern float mastCompassDeg;
#ifdef CMPS14
extern byte calibrationStatus[];
#endif
extern float mastDelta;

extern int num_n2k_messages;
extern int num_wind_messages;
extern int num_wind_other;
extern int headingErrCount;
extern int num_wind_fail;
extern int num_wind_other_fail;
extern int num_wind_other_ok;
extern unsigned long otherPGN[MAXPGN];
extern int otherPGNindex;

extern int num_0183_messages;
extern int num_0183_fail;
extern int num_0183_ok;

void mastHeading();
float readCompassDelta();
extern int mastAngle[2]; // array for both sensors
// 0 = honeywell
// 1 = compass


extern bool displayOnToggle, honeywellOnToggle;
/* bool teleplot=false;
extern int numReports[], totalReports;
*/

extern elapsedMillis time_since_last_mastcomp_rx;

#ifdef RS485CAN
void WindSpeed();
#else
void WindSpeed(const tN2kMsg &N2kMsg);
#endif
void BoatSpeed(const tN2kMsg &N2kMsg);

#ifdef BNO08X
#include <Adafruit_BNO08x.h>
extern Adafruit_BNO08x bno08x;
#endif
extern Preferences preferences;

extern File consLog;

void logTo::logToAll(String s) {
  if (s.endsWith("\n")) s.remove(s.length() - 1);
  String t = "[" + String(millis() / 1000) + "]: ";
  Serial.println(t + s);
  //consLog.println(s);
  if (serverStarted)
    WebSerial.println(t + s);
  s = t = String();
}

String logTo::commandList[] = {"?", "format", "restart", "ls", "scan", "status", "readings", "mast", "lsap", "toggle",
  "gps", "webserver", "compass", "windrx", "espnow", "teleplot", "hostname", "rtype", "n2k", "wifi", "rtk"};
String words[10];

void lsAPconn() {
  logTo::logToAll("AP connections");
  wifi_sta_list_t wifi_sta_list;
  tcpip_adapter_sta_list_t adapter_sta_list;
 
  memset(&wifi_sta_list, 0, sizeof(wifi_sta_list));
  memset(&adapter_sta_list, 0, sizeof(adapter_sta_list));
 
  esp_wifi_ap_get_sta_list(&wifi_sta_list);
  tcpip_adapter_get_sta_list(&wifi_sta_list, &adapter_sta_list);
 
  logTo::logToAll("stations: " + String(adapter_sta_list.num));
  for (int i = 0; i < adapter_sta_list.num; i++) {
    tcpip_adapter_sta_info_t station = adapter_sta_list.sta[i];
    logTo::logToAll("station nr " + String(i) + " MAC:");
    String printS;
    for(int i = 0; i< 6; i++){
      sprintf(prbuf, "%02X", station.mac[i]);
      printS += prbuf;
      if(i<5) printS += ".";
    }
    logTo::logToAll(printS);
    //Serial.print("IP: ");  
    byte octet[4];
    octet[3] = station.ip.addr & 0xFF;
    octet[2] = (station.ip.addr >> 8) & 0xFF;
    octet[1] = (station.ip.addr >> 16) & 0xFF;
    octet[0] = (station.ip.addr >> 24) & 0xFF;
    printS = String(octet[3]) + "." + String(octet[2]) + "." + String(octet[1]) + "." + String(octet[0]);
    logTo::logToAll(printS);
    printS = String();
    }
}

void i2cScan(TwoWire Wire) {
  byte error, address;
  int nDevices = 0;
  logTo::logToAll("Scanning...");
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission(); 
    char buf[16];
    sprintf(buf, "%2X", address); // Formats value as uppercase hex
    if (error == 0) {
      logTo::logToAll("I2C device found at address 0x" + String(buf));
      nDevices++;
    }
    else if (error == 4) {
      logTo::logToAll("error at address 0x" + String(buf));
    }
  }
  if (nDevices == 0) {
    logTo::logToAll("No I2C devices found\n");
  }
  else {
    logTo::logToAll("done\n");
  }
}

String doubleToTimeString(double time) {
  int hours = (int)(time / 10000);
  int minutes = (int)((time - hours * 10000) / 100);
  double seconds = fmod(time, 100);

  char timeStr[13];
  sprintf(timeStr, "%02d:%02d:%06.3f", hours, minutes, seconds);
  return String(timeStr);
}

void WebSerialonMessage(uint8_t *data, size_t len) {
  //Serial.printf("Received %lu bytes from WebSerial: ", len);
  Serial.write(data, len);
  //Serial.println();
  ////Serial.printf("commandList size is: %d\n", ASIZE(commandList));
  logTo::logToAll("Received Data...");
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
    logTo::logToAll(words[i]); 
    int j;
    if (words[i].equals("?")) {
      for (j = 1; j < logTo::ASIZE; j++) {
        logTo::logToAll(String(j) + ":" + logTo::commandList[j]);
      }
      return;
    }
    if (words[i].toInt() > 0)
      for (j = 1; j < logTo::ASIZE; j++) {
        //logTo::logToAll("j: " + String(j) + " " + logTo::commandList[j]);
        if (words[i].toInt() == j) {
          //Serial.printf("match %d %s %d %s\n", i, words[i].c_str(), j, commandList[j].c_str());
          words[i] = logTo::commandList[j];
        }
      }
    if (words[i].equals("status")) {
      logTo::logToAll("             uptime: " + String(millis() / 1000));
      logTo::logToAll("           AWS (in): " + String(WindSensor::windSpeedKnots));
      logTo::logToAll("           AWA (in): " + String(WindSensor::windAngleDegrees));
#ifdef HONEY
      logTo::logToAll(" Sensor L/H/Current: " + String(PotLo) + "/" + String(PotHi) + "/" + String(PotValue));
      logTo::logToAll("       Sensor angle: " + String(mastRotate));
      logTo::logToAll("        Correct AWA: " + String(rotateout));
#endif
      WebSerial.flush();
    }
    if (words[i].equals("compass")) {
      logTo::logToAll("               Boat IMU: " + String(compass.boatIMU));
      logTo::logToAll("           Boat Compass: " + String(compass.boatHeading));
      logTo::logToAll("      heading err count: " + String(headingErrCount));
      logTo::logToAll("           Mast Compass: " + String(mastCompassDeg));
#ifdef BNO08XXXXX
      logTo::logToAll("           Cal Status: " + String()
#endif
#ifdef CMPS14
      logTo::logToAll("            [Calibration]: ");
      String CV = String((uint16_t)((calibrationStatus[0] << 8) | calibrationStatus[1]));
      logTo::logToAll("mag: " + String(calibrationStatus[0]) + String(calibrationStatus[1]) + " " + CV);
      CV = String((calibrationStatus[2] << 8) | calibrationStatus[3]);
      logTo::logToAll("acc: " + String(calibrationStatus[2]) + String(calibrationStatus[3]) + " " + CV);
      // gyro cal is "currently broken"
      CV = String((calibrationStatus[6] << 8) | calibrationStatus[7]);
      logTo::logToAll("sys: " + String(calibrationStatus[6]) + String(calibrationStatus[7]) + " " + CV);
      logTo::logToAll();
      CV = String();
#endif
    }
    if (words[i].equals("n2k")) {
#ifdef N2K
      logTo::logToAll("           n2k main: " + String(num_n2k_messages));
      logTo::logToAll("           n2k wind: " + String(num_wind_messages));
      logTo::logToAll("      n2k wind fail: " + String(num_wind_fail));
      logTo::logToAll("       n2k wind fwd: " + String(num_wind_other));
      logTo::logToAll("  n2k wind fwd fail: " + String(num_wind_other_fail));
      logTo::logToAll("    n2k wind fwd ok: " + String(num_wind_other_ok));
      if (otherPGNindex > 0) {
        logTo::logToAll("n2k other PGNs: ");
        for (int i=0; i<otherPGNindex; i++) {
          logTo::logToAll(String(otherPGN[i]));
        }
      }
#endif
    }
    if (words[i].equals("format")) {
      SPIFFS.format();
      logTo::logToAll("SPIFFS formatted");
    }
    if (words[i].equals("restart")) {
      logTo::logToAll("Restarting...");
      ESP.restart();
    }
    if (words[i].equals("ls")) {
      File root = SPIFFS.open("/");
      File file = root.openNextFile();
      while (file) {
        logTo::logToAll(file.name());
        file.close(); 
        file = root.openNextFile();
      }
      root.close();
      //logTo::logToAll("done");
    }
    if (words[i].equals("scan")) {
      i2cScan(Wire);
    }
    if (words[i].equals("readings")) {
      //logTo::logToAll(readings);
    }
    if (words[i].equals("mast")) {
      //sendMastControl();
      //logTo::logToAll(readings);
    }
    if (words[i].equals("lsap")) {
      lsAPconn();
    }
    if (words[i].equals("wifi")) {
      int priority;
      String ssid, passwd;
      if (!words[++i].isEmpty()) {
        int j=i;
        while (!words[j].isEmpty()) {
          if (words[j].equals("-p")) { // priority (1..3)
            priority = words[++j].toInt();
          } 
          if (words[j].equals("-S")) { // ssid
            ssid = words[++j];
          }
          if (words[j].equals("-P")) { // password
            passwd = words[++j];
          }
          j++;
        } // while
        if (priority > 0 && priority < 4 && !ssid.isEmpty() && !passwd.isEmpty()) {
          logTo::logToAll("wifi params: -p: " + String(priority) + " -s: " + ssid + " -P: " + passwd);
        } else logTo::logToAll("wifi syntax -p <1..3> -S ssid -P password");
      } else { // no parameters
        if (WiFi.status() == WL_CONNECTED)
          logTo::logToAll("connected to: " + WiFi.SSID());  
        else logTo::logToAll("wifi not connected");
      }
      return;      
    }
    if (words[i].startsWith("tog")) {
      if (!words[++i].isEmpty()) {
        if (words[i].startsWith("disp")) {
          displayOnToggle = !displayOnToggle;
          logTo::logToAll("Display: " + String(displayOnToggle));
          return;
        }
        if (words[i].startsWith("comp")) {
          compass.OnToggle = !compass.OnToggle;
          logTo::logToAll("Compass: " + String(compass.OnToggle));
          return;
        }        
        if (words[i].startsWith("honey")) {
          honeywellOnToggle = !honeywellOnToggle;
          logTo::logToAll("Honeywell: " + String(honeywellOnToggle));
          return;
        }

      } else {
        logTo::logToAll("Display: " + String(displayOnToggle));
        logTo::logToAll("Compass: " + String(compass.OnToggle));
        logTo::logToAll("Honeywell: " + String(honeywellOnToggle));
      }
      return;
    }
#ifdef NMEA0183
    if (words[i].equals("gps") && pBD) {
      //Serial.printf("gps coords: %2.2d %2.2d\n", pBD->Latitude, pBD->Longitude);
      logTo::logToAll("Latitude: " + String(pBD->Latitude));
      logTo::logToAll("Longitude: " + String(pBD->Longitude));
      logTo::logToAll("Variation: " + String(pBD->Variation));
      logTo::logToAll("0183: " + String(num_0183_messages));
      logTo::logToAll("0183 fail: " + String(num_0183_fail));
      logTo::logToAll("0183 ok: " + String(num_0183_ok));
      return;
    }
#endif
#ifdef RTK
    if (words[i].equals("rtk") && pRTK) {
      logTo::logToAll("antA:" + String(pRTK->antennaAstat));
      logTo::logToAll("antB: " + String(pRTK->antennaBstat));
      logTo::logToAll("baselen: " + String(pRTK->baseLen));
      //logTo::logToAll("GPStime: " + String(pRTK->GPStime));
      logTo::logToAll("GPStime: " + doubleToTimeString(pRTK->GPStime));
      switch (pRTK->RTKqual) {
        case 0:
          logTo::logToAll("qual: no fix");
          break;
        case 4:
          logTo::logToAll("qual: RTK fix");
          break;        
        case 6:
          logTo::logToAll("qual: estimate (DR) mode");
          break;
      }
      logTo::logToAll("pitch/roll: " + String(pRTK->pitch) + " " + String(pRTK->roll));
      logTo::logToAll("heading: " + String(pRTK->heading));
      logTo::logToAll("pAcc/rAcc: " + String(pRTK->pAcc) + " " + String(pRTK->rAcc));
      logTo::logToAll("hAcc: " + String(pRTK->hAcc));
      logTo::logToAll("usedSV: " + String(pRTK->usedSV));      
      return;
    }
#endif
#ifdef PICAN
    if (bmeFound && words[i].startsWith("weat") && pENV) {
      logTo::logToAll("Cabin Temp: " + String(pENV->temp));
      logTo::logToAll("Pressure: " + String(pENV->pressure));
      logTo::logToAll("Humidity: " + String(pENV->humidity));
      return;
    }
#endif
    if (words[i].equals("webserver")) {
      logTo::logToAll("local IP: ");
      logTo::logToAll(WiFi.localIP().toString());
      logTo::logToAll("AP IP address: ");
      logTo::logToAll(WiFi.softAPIP().toString());
        int clientCount = WiFi.softAPgetStationNum();
        if (clientCount > 0) {
          logTo::logToAll("Clients connected: " + clientCount);
        } else {
          logTo::logToAll("No clients connected");
        }

    }
    if (words[i].equals("windrx")) {
      logTo::logToAll("last wind time: " + String(time_since_last_wind_rx) + " avg wind time: " + String(avg_time_since_last_wind) + " ms");
      if (time_since_last_wind_rx > 0.0)
        logTo::logToAll(String(1000.0/avg_time_since_last_wind) + " Hz (confirm timing 1000?)");
    }    
    if (words[i].equals("teleplot")) {
      compass.teleplot = !compass.teleplot;
      logTo::logToAll("teleplot: " + String(compass.teleplot));
      return;
    }
    if (words[i].equals("hostname")) {
      if (!words[++i].isEmpty()) {
        host = words[i];
        preferences.putString("hostname", host);
        logTo::logToAll("hostname set to " + host + "\n");
        logTo::logToAll("restart to change hostname\n");
        logTo::logToAll("preferences " + preferences.getString("hostname"));
      } else {
        logTo::logToAll("hostname: " + host);
      }
      return;
    }  
    if (words[i].equals("hall")) {
      logTo::logToAll("got true heading trigger, setting orientation mast: " + String(mastCompassDeg) + " boat: " + String(compass.boatIMU) + " delta: " + String(mastDelta));
      mastOrientation = 0;
      mastOrientation = mastDelta = readCompassDelta(); 
      logTo::logToAll("new orientation: " + String(mastDelta));
    return;
    }
#if 0
    if (words[i].startsWith("freq")) {
      int frequency = compassParams.frequency;
      if (!words[++i].isEmpty()) {
        frequency = atoi(words[i].c_str());
        if (frequency < BNOREADRATE) frequency = BNOREADRATE;
        preferences.putInt("frequency", frequency);
        compassParams.frequency = frequency;
      }
      logTo::logToAll("frequency %d\n", frequency);
      return;
    }
    if (words[i].startsWith("orient")) {
      if (!words[++i].isEmpty()) {
        int orientation = atoi(words[i].c_str());
        if (words[i].startsWith("+")) {
          words[i].remove(0, 1);
          orientation = compassParams.orientation + atoi(words[i].c_str());
        } else if (words[i].startsWith("-")) {
          words[i].remove(0, 1);
          orientation = compassParams.orientation - atoi(words[i].c_str());
        } else // no + or - so set orientation absolute
          compassParams.orientation = orientation;
        if (orientation < 0) orientation = 0;
        if (orientation > 359) orientation = 359;
        compassParams.orientation = orientation;
        preferences.putInt("orientation", orientation);
      } 
      logTo::logToAll("compass orientation %d\n",compassParams.orientation);
      return;
    }
#endif
    if (words[i].startsWith("report")) {
      for (int i=0; i<SH2_MAX_SENSOR_ID; i++) {
        if (compass.numReports[i] > 0)
          logTo::logToAll("report 0x" + String(i,HEX) + "/" + String(i) + ": " + String(compass.numReports[i]));
      }
      logTo::logToAll("total reports " + String(compass.totalReports));
      return;
    }
#ifdef BNO08X
    if (words[i].equals("rtype")) {
      int reportType;
      if (!words[++i].isEmpty()) {
        reportType = (int)strtol(words[i].c_str(), NULL, 16);
        preferences.putInt("rtype", reportType);
        compass.reportType = reportType;
        logTo::logToAll("compass report type set to 0x" + String(reportType,HEX));
        if (!compass.setReports()) 
                logTo::logToAll("Could not enable local report 0x" + String(reportType,HEX));
      } else {
        logTo::logToAll("compass report type is 0x" + String(reportType,HEX));
      }
      return;
    }
#endif
  }
  for (int i=0; i<wordCount; i++) words[i] = String();
  dataS = String();
}

