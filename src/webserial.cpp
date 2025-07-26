
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
//extern int mastRotate, rotateout;
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

#if 0
// mast compass
int convertMagHeading(const tN2kMsg &N2kMsg); // magnetic heading of boat (from e.g. B&G compass)
float parseMastHeading(const tN2kMsg &N2kMsg);  // mast heading
float parseN2KHeading(const tN2kMsg &N2kMsg); // boat heading from external source
float getCompass(int correction);      // boat heading from internal ESP32 CMPS14
void httpInit(const char* serverName);
extern const char* serverName;
extern int mastOrientation;   // delta between mast compass and boat compass
extern float mastCompassDeg;
#endif
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

extern Adafruit_SSD1306 *display; // temp hack; move to windparse.h, but requires a bunch of #ifdefs
int displayBright = 200;

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

void log::toAll(String s) {
  if (s.endsWith("\n")) s.remove(s.length() - 1);
  Serial.println(s);
  String t = "[" + String(millis() / 1000) + "]: " + s;
  if (consLog) consLog.println(t);
  //if (serverStarted)
    WebSerial.println(s);
  s = String();
  t = String();
}

String log::commandList[] = {"?", "format", "restart", "ls", "scan", "status", "readings", "mast", "lsap", "toggle",
  "gps", "gpsdebug", "webserver", "compass", "windrx", "espnow", "teleplot", "hostname", "rtype", "n2k", "wifi", "rtk", "gsv", 
"tuning"};

const char *RTKqualStr[] = {"invalid", "single point", "differential GPS", "RTK fix", "RTK float", "DR", "manual", "xtra wide", "SBAS"};

String words[10];

void lsAPconn() {
  log::toAll("AP connections");
  wifi_sta_list_t wifi_sta_list;
  tcpip_adapter_sta_list_t adapter_sta_list;
 
  memset(&wifi_sta_list, 0, sizeof(wifi_sta_list));
  memset(&adapter_sta_list, 0, sizeof(adapter_sta_list));
 
  esp_wifi_ap_get_sta_list(&wifi_sta_list);
  tcpip_adapter_get_sta_list(&wifi_sta_list, &adapter_sta_list);
 
  log::toAll("stations: " + String(adapter_sta_list.num));
  for (int i = 0; i < adapter_sta_list.num; i++) {
    tcpip_adapter_sta_info_t station = adapter_sta_list.sta[i];
    log::toAll("station nr " + String(i) + " MAC:");
    String printS;
    for(int i = 0; i< 6; i++){
      sprintf(prbuf, "%02X", station.mac[i]);
      printS += prbuf;
      if(i<5) printS += ".";
    }
    log::toAll(printS);
    //Serial.print("IP: ");  
    byte octet[4];
    octet[3] = station.ip.addr & 0xFF;
    octet[2] = (station.ip.addr >> 8) & 0xFF;
    octet[1] = (station.ip.addr >> 16) & 0xFF;
    octet[0] = (station.ip.addr >> 24) & 0xFF;
    printS = String(octet[3]) + "." + String(octet[2]) + "." + String(octet[1]) + "." + String(octet[0]);
    log::toAll(printS);
    printS = String();
    }
}

void i2cScan(TwoWire Wire) {
  byte error, address;
  int nDevices = 0;
  log::toAll("Scanning i2c...");
  for (address = 1; address < 127; address++) {
    Serial.printf("0x",address);
    Wire.beginTransmission(address);
    error = Wire.endTransmission(); 
    char buf[16];
    sprintf(buf, "%2X", address); // Formats value as uppercase hex
    if (error == 0) {
      log::toAll("I2C device found at address 0x" + String(buf));
      nDevices++;
    }
    else if (error == 4) {
      log::toAll("error at address 0x" + String(buf));
    }
  }
  if (nDevices == 0) {
    log::toAll("No I2C devices found");
  }
  else {
    log::toAll("done");
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
  //Serial.write(data, len);
  //Serial.println();
  //Serial.printf("commandList size is: %d\n", ASIZE(commandList));
  String dataS = String((char*)data);
  log::toAll(dataS);
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
  //log::toAll("words: " + String(wordCount));
  for (int i = 0; i < wordCount; i++) {   
    //log::toAll(String(i) + ":" + words[i]); 
    int j;
    if (words[i].equals("?")) {
      for (j = 1; j < log::ASIZE; j++) {
        log::toAll(String(j) + ":" + log::commandList[j]);
      }
      return;
    }
    if (words[i].toInt() > 0)
      for (j = 1; j < log::ASIZE; j++) {
        //log::toAll("j: " + String(j) + " " + log::commandList[j]);
        if (words[i].toInt() == j) {
          //Serial.printf("match %d %s %d %s\n", i, words[i].c_str(), j, commandList[j].c_str());
          words[i] = log::commandList[j];
        }
      }
    if (words[i].equals("status")) {
      log::toAll("             uptime: " + String(millis() / 1000));
      log::toAll("           AWS (in): " + String(WindSensor::windSpeedKnots));
      log::toAll("           AWA (in): " + String(WindSensor::windAngleDegrees));
      //WebSerial.flush();
#ifdef HONEY
      log::toAll(" Sensor L/H/Current: " + String(PotLo) + "/" + String(PotHi) + "/" + String(PotValue));
      log::toAll("       Sensor angle: " + String(mastRotate));
      log::toAll("        Correct AWA: " + String(rotateout));
#endif
      return;
    }
    if (words[i].equals("wind")) {
      log::toAll("STW: " + String(pBD->STW*MTOKTS));
      log::toAll("AWS: " + String(WindSensor::windSpeedKnots) + " AWA: " + String(WindSensor::windAngleDegrees));
      log::toAll("TWS: " + String(pBD->TWS*MTOKTS) + " TWA: " + String(pBD->TWA));
      log::toAll("HDG: " + String(pBD->trueHeading));
      log::toAll("TWD:" + String(pBD->TWD));
      log::toAll("VMG: " + String(pBD->VMG*MTOKTS));
      log::toAll("maxTWS: " + String(pBD->maxTWS*MTOKTS));
      return;
    }
    if (words[i].equals("potlog")) {
      logPot = !logPot;
      log::toAll("logPot: " + String(logPot));
      return;
    }
    if (words[i].equals("compass")) {
      log::toAll("           Boat Compass: " + String(compass.boatHeading));
      log::toAll("              Boat True: " + String(pBD->trueHeading));
      log::toAll("              Variation: " + String(pBD->Variation*RADTODEG));
      log::toAll("            Orientation: " + String(boatOrientation));
      log::toAll("      Heading Err Count: " + String(headingErrCount));
#ifdef BNO_GRV
      log::toAll("               Boat IMU: " + String(compass.boatIMU));
      log::toAll("           Mast Compass: " + String(mastCompassDeg));
#ifdef BNO08XXXXX
      log::toAll("           Cal Status: " + String()
#endif
#ifdef CMPS14
      log::toAll("            [Calibration]: ");
      String CV = String((uint16_t)((calibrationStatus[0] << 8) | calibrationStatus[1]));
      log::toAll("mag: " + String(calibrationStatus[0]) + String(calibrationStatus[1]) + " " + CV);
      CV = String((calibrationStatus[2] << 8) | calibrationStatus[3]);
      log::toAll("acc: " + String(calibrationStatus[2]) + String(calibrationStatus[3]) + " " + CV);
      // gyro cal is "currently broken"
      CV = String((calibrationStatus[6] << 8) | calibrationStatus[7]);
      log::toAll("sys: " + String(calibrationStatus[6]) + String(calibrationStatus[7]) + " " + CV);
      log::toAll();
      CV = String();
#endif
#endif
      return;
    } // compass
#ifdef N2K
    if (words[i].equals("n2k")) {
      log::toAll("           n2k main: " + String(num_n2k_messages));
      log::toAll("           n2k wind: " + String(num_wind_messages));
      log::toAll("      n2k wind fail: " + String(num_wind_fail));
      log::toAll("       n2k wind fwd: " + String(num_wind_other));
      log::toAll("  n2k wind fwd fail: " + String(num_wind_other_fail));
      log::toAll("    n2k wind fwd ok: " + String(num_wind_other_ok));
      if (otherPGNindex > 0) {
        log::toAll("n2k other PGNs: ");
        for (int i=0; i<otherPGNindex; i++) {
          log::toAll(String(otherPGN[i]));
        }
      }
      return;
    }
#endif
    if (words[i].equals("format")) {
      SPIFFS.format();
      log::toAll("SPIFFS formatted");
      return;
    }
    if (words[i].equals("restart")) {
      log::toAll("Restarting...");
      ESP.restart();
    }
    if (words[i].equals("ls")) {
      File root = SPIFFS.open("/");
      File file = root.openNextFile();
      while (file) {
        log::toAll(file.name());
        file.close(); 
        file = root.openNextFile();
      }
      root.close();
      //log::toAll("done");
      return;
    }
    if (words[i].equals("scan")) {
      i2cScan(Wire);
      return;
    }
    if (words[i].equals("readings")) {
      //log::toAll(readings);
      return;
    }
    if (words[i].equals("mast")) {
      //sendMastControl();
      //log::toAll(readings);
      return;
    }
    if (words[i].equals("lsap")) {
      lsAPconn();
      return;
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
          log::toAll("wifi params: -p: " + String(priority) + " -s: " + ssid + " -P: " + passwd);
        } else log::toAll("wifi syntax -p <1..3> -S ssid -P password");
      } else { // no parameters
        if (WiFi.status() == WL_CONNECTED)
          log::toAll("connected to: " + WiFi.SSID());  
        else log::toAll("wifi not connected");
      }
      return;      
    }
    if (words[i].startsWith("tog")) {
      if (!words[++i].isEmpty()) {
        if (words[i].startsWith("disp")) {
          displayOnToggle = !displayOnToggle;
          log::toAll("Display: " + String(displayOnToggle));
          return;
        }
//#ifdef BNO_GRV
        if (words[i].startsWith("comp")) {
          compass.OnToggle = !compass.OnToggle;
          log::toAll("Compass: " + String(compass.OnToggle));
          return;
        }        
//#endif
        if (words[i].startsWith("honey")) {
          honeywellOnToggle = !honeywellOnToggle;
          log::toAll("Honeywell: " + String(honeywellOnToggle));
          return;
        }
        if (words[i].startsWith("stack")) {
          stackTrace = !stackTrace;
          log::toAll("stackTrace: " + String(stackTrace));
          return;
        }
        if (words[i].startsWith("windlo")) {
          windLogging = !windLogging;
          if (!windLogging) startNextWindLog();
          log::toAll("windLogging: " + String(windLogging));
          return;
        }
      } else {
        log::toAll("Display: " + String(displayOnToggle));
//#ifdef BNO_GRV
        log::toAll("Compass: " + String(compass.OnToggle));
//#endif
        log::toAll("Honeywell: " + String(honeywellOnToggle));
        log::toAll("stackTrace: " + String(stackTrace));
        log::toAll("Wind Log: " + String(windLogging));
      }
      return;
    }
#ifdef NMEA0183
    if (words[i].equals("gps") && pBD) {
      //Serial.printf("gps coords: %2.2d %2.2d\n", pBD->Latitude, pBD->Longitude);
      log::toAll("Latitude: " + String(pBD->Latitude));
      log::toAll("Longitude: " + String(pBD->Longitude));
      log::toAll("Variation: " + String(pBD->Variation*RADTODEG));
      // heading is degrees !!!
      log::toAll("Heading: " + String(pBD->trueHeading));
      log::toAll("COG: " + String(pBD->COG*RADTODEG));
      log::toAll("SOG: " + String(pBD->SOG));
      log::toAll("0183 fail: " + String(num_0183_fail));
      log::toAll("0183 ok: " + String(num_0183_ok));
      return;
    }
#if 0
    if (words[i].startsWith("gpsde") && pBD) {
      gpsDebug = !gpsDebug;
      log::toAll("rtkDebug: " + String(rtkDebug));
    }
#endif
#endif
#ifdef RTK
    if (words[i].equals("rtk")) {
      if (wordCount > 1 && words[++i].equals("debug")) {
        rtkDebug = !rtkDebug;
        log::toAll("rtkDebug: " + String(rtkDebug));
        return;
      } else if (pRTK) {
        //log::toAll("antA:" + String(pRTK->antennaAstat));
        //log::toAll("antB: " + String(pRTK->antennaBstat));
        //log::toAll("baselen: " + String(pRTK->baseLen));
        //log::toAll("GPStime: " + String(pRTK->GPStime));
        log::toAll("GPStime: " + doubleToTimeString(pRTK->GPStime));
        // TBD: fix this. Depends on which GPS sensor
#if 0
        switch (pRTK->RTKqual) {
          case 0:
            log::toAll("qual: no fix");
            break;
          case 4:
            log::toAll("qual: RTK fix");
            break;        
          case 6:
            log::toAll("qual: estimate (DR) mode");
            break;
        }
#endif
        log::toAll("RTK fix quality: " + String(RTKqualStr[pRTK->RTKqual]));
        log::toAll("pitch/roll: " + String(pRTK->pitch) + " " + String(pRTK->roll));
        log::toAll("heading: " + String(pRTK->heading));
        log::toAll("RTK orientation: " + String(rtkOrientation));
        //log::toAll("pAcc/rAcc: " + String(pRTK->pAcc) + " " + String(pRTK->rAcc));
        //log::toAll("hAcc: " + String(pRTK->hAcc));
        log::toAll("usedSV: " + String(pRTK->usedSV));      
        return;
      }
    }
    if (words[i].equals("tuning")) {
      tuning = !tuning;
    }
#endif
#ifdef PICAN
    if (bmeFound && words[i].startsWith("weat") && pENV) {
      log::toAll("Cabin Temp: " + String(pENV->temp));
      log::toAll("Pressure: " + String(pENV->pressure));
      log::toAll("Humidity: " + String(pENV->humidity));
      return;
    }
#endif
    if (words[i].equals("webserver")) {
      log::toAll("local IP: ");
      log::toAll(WiFi.localIP().toString());
      log::toAll("AP IP address: ");
      log::toAll(WiFi.softAPIP().toString());
        int clientCount = WiFi.softAPgetStationNum();
        if (clientCount > 0) {
          log::toAll("Clients connected: " + clientCount);
        } else {
          log::toAll("No clients connected");
        }
      return;
    }
    if (words[i].equals("windrx")) {
      log::toAll("last wind time: " + String(time_since_last_wind_rx) + " avg wind time: " + String(avg_time_since_last_wind) + " ms");
      if (time_since_last_wind_rx > 0.0)
        log::toAll(String(1000.0/avg_time_since_last_wind) + " Hz (confirm timing 1000?)");
      return;
    }    
#ifdef BNO08X
    if (words[i].equals("teleplot")) {
      compass.teleplot = !compass.teleplot;
      log::toAll("teleplot: " + String(compass.teleplot));
      return;
    }
#endif
    if (words[i].equals("hostname")) {
      if (!words[++i].isEmpty()) {
        host = words[i];
        preferences.putString("hostname", host);
        log::toAll("hostname set to " + host + "\n");
        log::toAll("restart to change hostname\n");
        log::toAll("preferences " + preferences.getString("hostname"));
      } else {
        log::toAll("hostname: " + host);
      }
      return;
    }  
    if (words[i].equals("variation")) {
      if (!words[++i].isEmpty()) {
        pBD->Variation = atof(words[i].c_str())*DEGTORAD;
        preferences.putFloat("variation", pBD->Variation);
        log::toAll("variation set to " + String(pBD->Variation));
      } else {
        log::toAll("variation: " + String(pBD->Variation));
      }
      return;
    }  
#ifdef BNO_GRV
    if (words[i].equals("hall")) {
      log::toAll("got true heading trigger, setting orientation mast: " + String(mastCompassDeg) + " boat: " + String(compass.boatIMU) + " delta: " + String(mastDelta));
      mastOrientation = 0;
      mastOrientation = mastDelta = readCompassDelta(); 
      log::toAll("new orientation: " + String(mastDelta));
      return;
    }
#endif
#ifdef PICAN
    if (words[i].equals("gsv")) {
      log::toAll("maxSat: " + String(maxSat));
      int totalSat = 0;
      for (int j=0; j<MAXSAT; j++) {
        if (GSVseen[j].SVID > 0) {
          totalSat++;
          log::toAll(String(GSVseen[j].SVID) + " el: " + String(GSVseen[j].Elevation) + " az: " + String(GSVseen[j].Azimuth) + " SNR: " + String(GSVseen[j].SNR));
          GSVseen[j].SVID = 0;
        }
        if (totalSat % 10 == 0) WebSerial.flush();
      }
      log::toAll("total satellites: " + String(totalSat));
      log::toAll("resetting GSV list!");
      return;
    }
    if (words[i].startsWith("gsvtog")) {
      GSVtoggle = !GSVtoggle;
      log::toAll("GSVtoggle: " + String(GSVtoggle));
      return;
    }
#endif
    if (words[i].startsWith("pass")) {
      passThrough = !passThrough;
      log::toAll("pass: " + String(passThrough?"true":"false"));
      WebSerial.printf("PT: %s\n",passThrough?"true":"false");
      return;
    }
#ifdef BNO_GRV
    if (words[i].startsWith("freq")) {
      int frequency = compassParams.frequency;
      if (!words[++i].isEmpty()) {
        frequency = atoi(words[i].c_str());
        if (frequency < BNOREADRATE) frequency = BNOREADRATE;
        preferences.putInt("frequency", frequency);
        compassParams.frequency = frequency;
      }
      log::toAll("frequency %d\n", frequency);
      return;
    }
#endif
#ifdef BNO08X
if (words[i].startsWith("freq")) {
  int frequency = compass.frequency;
  if (!words[++i].isEmpty()) {
    frequency = atoi(words[i].c_str());
    //if (frequency < BNOREADRATE) frequency = BNOREADRATE;
    preferences.putInt("frequency", frequency);
    compass.frequency = frequency;
  }
  log::toAll("frequency " + String(frequency));
  return;
}
#endif
#if 1
    if (words[i].startsWith("orient")) {
      if (!words[++i].isEmpty()) {
        int orientation = atoi(words[i].c_str());
        if (words[i].startsWith("+")) {
          words[i].remove(0, 1);
          orientation = boatOrientation + atoi(words[i].c_str());
        } else if (words[i].startsWith("-")) {
          words[i].remove(0, 1);
          orientation = boatOrientation - atoi(words[i].c_str());
        } // no + or - so set orientation absolute
        if (orientation < 0) orientation = 0;
        if (orientation > 359) orientation = 359;
        boatOrientation = orientation;
        preferences.putInt("orientation", orientation);
      } 
      log::toAll("boat compass orientation: " + String(boatOrientation));
      return;
    }
#endif
//#ifdef BNO_GRV
    if (words[i].startsWith("report")) {
      for (int i=0; i<SH2_MAX_SENSOR_ID; i++) {
        if (compass.numReports[i] > 0)
          log::toAll("report 0x" + String(i,HEX) + "/" + String(i) + ": " + String(compass.numReports[i]));
      }
      log::toAll("total reports " + String(compass.totalReports));
      return;
    }
//#endif
#ifdef BNO08X
    if (words[i].equals("rtype")) {
      int reportType;
      if (!words[++i].isEmpty()) {
        reportType = (int)strtol(words[i].c_str(), NULL, 16);
        preferences.putInt("rtype", reportType);
        compass.reportType = reportType;
        log::toAll("compass report type set to 0x" + String(reportType,HEX));
        if (!compass.setReports()) 
                log::toAll("Could not enable local report 0x" + String(reportType,HEX));
      } else {
        log::toAll("compass report type is 0x" + String(reportType,HEX));
      }
      return;
    }
#endif
#if 0 // maybe change to a screen timeout but how to turn back on without web interface?
    if (words[i].equals("bright")) {
      i++;
      if (words[i].equals("up")) {
        displayBright += 10;
        if (displayBright > 255) displayBright = 255;
      } else if (words[i].equals("down")) {
        displayBright -= 10;
        if (displayBright < 0) displayBright = 0;
      }
      log::toAll("set contrast");
      display->ssd1306_command(SSD1306_SETCONTRAST);
      display->ssd1306_command(0);
      return;
    }
#endif
  }
  for (int i=0; i<wordCount; i++) words[i] = String();
  dataS = String();
}

