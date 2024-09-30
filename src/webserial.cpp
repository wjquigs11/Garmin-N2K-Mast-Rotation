<<<<<<< HEAD

#include "windparse.h"
#include "BoatData.h"

// object-oriented classes
#include "BNO085Compass.h"
#include "logto.h"

=======
#include "windparse.h"
#include <Adafruit_ADS1X15.h>
#include "BoatData.h"

>>>>>>> 6b69be646e028764cc6b07b27ebd843c0deca33f
extern tBoatData *pBD;
extern tBoatData BoatData;
// TBD: I should add mag heading, mast heading etc (all the globals) to BoatData struct so I could have a single extern in each file

extern Stream *forward_stream;

extern tNMEA2000 *n2kMain;

<<<<<<< HEAD
#ifdef HONEY
=======
>>>>>>> 6b69be646e028764cc6b07b27ebd843c0deca33f
// Honeywell sensor
extern movingAvg honeywellSensor;                // define the moving average object
extern int mastRotate, rotateout;
extern int PotValue, PotLo, PotHi;
extern Adafruit_ADS1015 ads;
extern int adsInit;
<<<<<<< HEAD
#endif
=======
>>>>>>> 6b69be646e028764cc6b07b27ebd843c0deca33f

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
extern float boatCompassDeg; // magnetic heading not corrected for variation
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

void mastHeading();
float readCompassDelta();
extern int mastAngle[2]; // array for both sensors
// 0 = honeywell
// 1 = compass

<<<<<<< HEAD

extern bool displayOnToggle, honeywellOnToggle;
/* bool teleplot=false;
extern int numReports[], totalReports;
*/
=======
extern bool displayOnToggle, compassOnToggle, honeywellOnToggle;
bool teleplot=false;
extern int numReports[], totalReports;
>>>>>>> 6b69be646e028764cc6b07b27ebd843c0deca33f

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

<<<<<<< HEAD
void logTo::logToAll(String s) {
  if (s.endsWith("\n")) s.remove(s.length() - 1);
  Serial.println(s);
  //consLog.println(s);
  if (serverStarted)
    WebSerial.println(s);
  s = String();
}

String logTo::commandList[logTo::ASIZE] = {"?", "format", "restart", "ls", "scan", "status", "readings", "mast", "lsap", "toggle",
  "gps", "webserver", "compass", "windrx", "espnow", "teleplot", "hostname", "rtype", "n2k", "wifi"};
String words[10];

void lsAPconn() {
  logTo::logToAll("AP connections");
=======
void logToAll(String s) {
  if (s.endsWith("\n")) s.remove(s.length() - 1);
  Serial.println(s);
  consLog.println(s);
  if (serverStarted) {
    WebSerial.println(s);
    WebSerial.flush();
  }
  s = String();
}

void lsAPconn() {
  logToAll("AP connections");
>>>>>>> 6b69be646e028764cc6b07b27ebd843c0deca33f
  wifi_sta_list_t wifi_sta_list;
  tcpip_adapter_sta_list_t adapter_sta_list;
 
  memset(&wifi_sta_list, 0, sizeof(wifi_sta_list));
  memset(&adapter_sta_list, 0, sizeof(adapter_sta_list));
 
  esp_wifi_ap_get_sta_list(&wifi_sta_list);
  tcpip_adapter_get_sta_list(&wifi_sta_list, &adapter_sta_list);
 
<<<<<<< HEAD
  logTo::logToAll("stations: " + String(adapter_sta_list.num));
  for (int i = 0; i < adapter_sta_list.num; i++) {
    tcpip_adapter_sta_info_t station = adapter_sta_list.sta[i];
    logTo::logToAll("station nr " + String(i) + " MAC:");
=======
  logToAll("stations: " + String(adapter_sta_list.num));
  for (int i = 0; i < adapter_sta_list.num; i++) {
    tcpip_adapter_sta_info_t station = adapter_sta_list.sta[i];
    logToAll("station nr " + String(i) + " MAC:");
>>>>>>> 6b69be646e028764cc6b07b27ebd843c0deca33f
    String printS;
    for(int i = 0; i< 6; i++){
      sprintf(prbuf, "%02X", station.mac[i]);
      printS += prbuf;
      if(i<5) printS += ".";
    }
<<<<<<< HEAD
    logTo::logToAll(printS);
=======
    logToAll(printS);
>>>>>>> 6b69be646e028764cc6b07b27ebd843c0deca33f
    //Serial.print("IP: ");  
    byte octet[4];
    octet[3] = station.ip.addr & 0xFF;
    octet[2] = (station.ip.addr >> 8) & 0xFF;
    octet[1] = (station.ip.addr >> 16) & 0xFF;
    octet[0] = (station.ip.addr >> 24) & 0xFF;
    printS = String(octet[3]) + "." + String(octet[2]) + "." + String(octet[1]) + "." + String(octet[0]);
<<<<<<< HEAD
    logTo::logToAll(printS);
=======
    logToAll(printS);
>>>>>>> 6b69be646e028764cc6b07b27ebd843c0deca33f
    printS = String();
    }
}

void i2cScan(TwoWire Wire) {
  byte error, address;
  int nDevices = 0;
<<<<<<< HEAD
  logTo::logToAll("Scanning...");
=======
  logToAll("Scanning...");
>>>>>>> 6b69be646e028764cc6b07b27ebd843c0deca33f
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission(); 
    char buf[16];
    sprintf(buf, "%2X", address); // Formats value as uppercase hex
    if (error == 0) {
<<<<<<< HEAD
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

=======
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

String commandList[] = {"?", "format", "restart", "ls", "scan", "status", "readings", "mast", "lsap", "toggle",
   "gps", "webserver", "compass", "windrx", "espnow", "teleplot", "hostname", "rtype", "n2k", "wifi", "hall"};
#define ASIZE(arr) (sizeof(arr) / sizeof(arr[0]))
String words[10]; // Assuming a maximum of 10 words

>>>>>>> 6b69be646e028764cc6b07b27ebd843c0deca33f
void WebSerialonMessage(uint8_t *data, size_t len) {
  //Serial.printf("Received %lu bytes from WebSerial: ", len);
  Serial.write(data, len);
  //Serial.println();
  ////Serial.printf("commandList size is: %d\n", ASIZE(commandList));
<<<<<<< HEAD
  WebSerial.println("Received Data...");
=======
  logToAll("Received Data...");
>>>>>>> 6b69be646e028764cc6b07b27ebd843c0deca33f
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
<<<<<<< HEAD
    WebSerial.println(words[i]); 
    int j;
    if (words[i].equals("?")) {
      for (j = 1; j < logTo::ASIZE; j++) {
        WebSerial.println(String(j) + ":" + logTo::commandList[j]);
      }
    }
    if (words[i].toInt() > 0)
      for (j = 1; j < logTo::ASIZE; j++) {
        //WebSerial.println("j: " + String(j) + " " + logTo::commandList[j]);
        if (words[i].toInt() == j) {
          //Serial.printf("match %d %s %d %s\n", i, words[i].c_str(), j, commandList[j].c_str());
          words[i] = logTo::commandList[j];
        }
      }
    if (words[i].equals("status")) {
      WebSerial.println("             uptime: " + String(millis() / 1000));
      WebSerial.println("           AWS (in): " + String(WindSensor::windSpeedKnots));
      WebSerial.println("           AWA (in): " + String(WindSensor::windAngleDegrees));
#ifdef HONEY
      WebSerial.println(" Sensor L/H/Current: " + String(PotLo) + "/" + String(PotHi) + "/" + String(PotValue));
      WebSerial.println("       Sensor angle: " + String(mastRotate));
      WebSerial.println("        Correct AWA: " + String(rotateout));
#endif
      WebSerial.println("       Mast Compass: " + String(mastCompassDeg));
      WebSerial.println("         Mast angle: " + String(mastDelta));
      WebSerial.flush();
    }
    if (words[i].equals("compass")) {
      WebSerial.println("           Boat Compass: " + String(boatCompassDeg));
      WebSerial.println("      heading err count: " + String(headingErrCount));
      WebSerial.println("           Mast Compass: " + String(mastCompassDeg));
#ifdef BNO08XXXXX
      WebSerial.println("           Cal Status: " + String()
#endif
#ifdef CMPS14
      logTo::logToAll("            [Calibration]: ");
      String CV = String((uint16_t)((calibrationStatus[0] << 8) | calibrationStatus[1]));
      WebSerial.println("mag: " + String(calibrationStatus[0]) + String(calibrationStatus[1]) + " " + CV);
      CV = String((calibrationStatus[2] << 8) | calibrationStatus[3]);
      WebSerial.println("acc: " + String(calibrationStatus[2]) + String(calibrationStatus[3]) + " " + CV);
      // gyro cal is "currently broken"
      CV = String((calibrationStatus[6] << 8) | calibrationStatus[7]);
      WebSerial.println("sys: " + String(calibrationStatus[6]) + String(calibrationStatus[7]) + " " + CV);
      WebSerial.println();
=======
    logToAll(words[i]); 
    int j;
    if (words[i].equals("?")) {
      for (j = 1; j < ASIZE(commandList); j++) {
        logToAll(String(j) + ":" + commandList[j]);
      }
    }
    if (words[i].toInt() > 0)
      for (j = 1; j < ASIZE(commandList); j++) {
        //logToAll("j: " + String(j) + " " + commandList[j]);
        if (words[i].toInt() == j) {
          //Serial.printf("match %d %s %d %s\n", i, words[i].c_str(), j, commandList[j].c_str());
          words[i] = commandList[j];
        }
      }
    if (words[i].equals("status")) {
      logToAll("             uptime: " + String(millis() / 1000));
      logToAll("           AWS (in): " + String(WindSensor::windSpeedKnots));
      logToAll("           AWA (in): " + String(WindSensor::windAngleDegrees));
      logToAll(" Sensor L/H/Current: " + String(PotLo) + "/" + String(PotHi) + "/" + String(PotValue));
      logToAll("       Sensor angle: " + String(mastRotate,2));
      logToAll("        Correct AWA: " + String(rotateout,2));
      logToAll("       Mast Compass: " + String(mastCompassDeg,2));
      logToAll("         Mast angle: " + String(mastDelta,2));
    }
    if (words[i].equals("compass")) {
      logToAll("           Boat Compass: " + String(boatCompassDeg));
      logToAll("      heading err count: " + String(headingErrCount));
      logToAll("           Mast Compass: " + String(mastCompassDeg));
#ifdef BNO08XXXXX
      logToAll("           Cal Status: " + String()
#endif
#ifdef CMPS14
      WebSerial.printf("            [Calibration]: ");
      String CV = String((uint16_t)((calibrationStatus[0] << 8) | calibrationStatus[1]));
      logToAll("mag: " + String(calibrationStatus[0]) + String(calibrationStatus[1]) + " " + CV);
      CV = String((calibrationStatus[2] << 8) | calibrationStatus[3]);
      logToAll("acc: " + String(calibrationStatus[2]) + String(calibrationStatus[3]) + " " + CV);
      // gyro cal is "currently broken"
      CV = String((calibrationStatus[6] << 8) | calibrationStatus[7]);
      logToAll("sys: " + String(calibrationStatus[6]) + String(calibrationStatus[7]) + " " + CV);
      logToAll();
>>>>>>> 6b69be646e028764cc6b07b27ebd843c0deca33f
      CV = String();
#endif
    }
    if (words[i].equals("n2k")) {
#ifdef N2K
<<<<<<< HEAD
      WebSerial.println("           n2k main: " + String(num_n2k_messages));
      WebSerial.println("           n2k wind: " + String(num_wind_messages));
      WebSerial.println("      n2k wind fail: " + String(num_wind_fail));
      WebSerial.println("       n2k wind fwd: " + String(num_wind_other));
      WebSerial.println("  n2k wind fwd fail: " + String(num_wind_other_fail));
      WebSerial.println("    n2k wind fwd ok: " + String(num_wind_other_ok));
      if (otherPGNindex > 0) {
        WebSerial.print("n2k other PGNs: ");
        for (int i=0; i<otherPGNindex; i++) {
          WebSerial.print(otherPGN[i]);
          if (i+1<otherPGNindex) WebSerial.print(", ");
        }
        WebSerial.println();
=======
      logToAll("           n2k main: " + String(num_n2k_messages));
      logToAll("           n2k wind: " + String(num_wind_messages));
      logToAll("      n2k wind fail: " + String(num_wind_fail));
      logToAll("       n2k wind fwd: " + String(num_wind_other));
      logToAll("  n2k wind fwd fail: " + String(num_wind_other_fail));
      logToAll("    n2k wind fwd ok: " + String(num_wind_other_ok));
      logToAll("      last mast rcv: " + String(time_since_last_mastcomp_rx));
      if (otherPGNindex > 0) {
        logToAll("n2k other PGNs: ");
        for (int i=0; i<otherPGNindex; i++) {
          logToAll(String(otherPGN[i]));
        }
>>>>>>> 6b69be646e028764cc6b07b27ebd843c0deca33f
      }
#endif
    }
    if (words[i].equals("format")) {
      SPIFFS.format();
<<<<<<< HEAD
      WebSerial.println("SPIFFS formatted");
    }
    if (words[i].equals("restart")) {
      WebSerial.println("Restarting...");
=======
      logToAll("SPIFFS formatted");
    }
    if (words[i].equals("restart")) {
      logToAll("Restarting...");
>>>>>>> 6b69be646e028764cc6b07b27ebd843c0deca33f
      ESP.restart();
    }
    if (words[i].equals("ls")) {
      File root = SPIFFS.open("/");
      File file = root.openNextFile();
      while (file) {
<<<<<<< HEAD
        WebSerial.println(file.name());
=======
        logToAll(file.name());
>>>>>>> 6b69be646e028764cc6b07b27ebd843c0deca33f
        file.close(); 
        file = root.openNextFile();
      }
      root.close();
<<<<<<< HEAD
      //WebSerial.println("done");
=======
      //logToAll("done");
>>>>>>> 6b69be646e028764cc6b07b27ebd843c0deca33f
    }
    if (words[i].equals("scan")) {
      i2cScan(Wire);
    }
    if (words[i].equals("readings")) {
<<<<<<< HEAD
      WebSerial.println(readings);
    }
    if (words[i].equals("mast")) {
      //sendMastControl();
      //WebSerial.println(readings);
=======
      logToAll(JSON.stringify(readings));
    }
    if (words[i].equals("mast")) {
      //sendMastControl();
      //logToAll(readings);
>>>>>>> 6b69be646e028764cc6b07b27ebd843c0deca33f
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
<<<<<<< HEAD
        if (priority > 0 && priority < 4 && !ssid.isEmpty() && !passwd.isEmpty()) {
          logTo::logToAll("wifi params: -p: " + String(priority) + " -s: " + ssid + " -P: " + passwd);
        } else logTo::logToAll("wifi syntax -p <1..3> -S ssid -P password");
      } else { // no parameters
        if (WiFi.status() == WL_CONNECTED)
          logTo::logToAll("connected to: " + WiFi.SSID());  
        else logTo::logToAll("wifi not connected");
=======
        if (priority > 0 && priority <= MAX_NETS && !ssid.isEmpty() && !passwd.isEmpty()) {
          logToAll("wifi params: -p: " + String(priority) + " -s: " + ssid + " -P: " + passwd);
          writeWiFi(priority-1, ssid, passwd);
        } else {
          logToAll("wifi syntax -p <1..3> -S ssid -P password");
          logToAll("current: -p: " + String(priority) + " -S: " + ssid + " -P: " + passwd);
        }
      } else { // no parameters
        if (WiFi.status() == WL_CONNECTED)
          logToAll("connected to: " + WiFi.SSID());  
        else logToAll("wifi not connected");
>>>>>>> 6b69be646e028764cc6b07b27ebd843c0deca33f
      }
      return;      
    }
    if (words[i].equals("toggle")) {
<<<<<<< HEAD
      WebSerial.println("Display: " + String(displayOnToggle));
      WebSerial.println("Compass: " + String(compass.OnToggle));
      WebSerial.println("Honeywell: " + String(honeywellOnToggle));
=======
      logToAll("Display: " + String(displayOnToggle));
      logToAll("Compass: " + String(compassOnToggle));
      logToAll("Honeywell: " + String(honeywellOnToggle));
>>>>>>> 6b69be646e028764cc6b07b27ebd843c0deca33f
    }
#ifdef NMEA0183
    if (words[i].equals("gps") && pBD) {
      //Serial.printf("gps coords: %2.2d %2.2d\n", pBD->Latitude, pBD->Longitude);
<<<<<<< HEAD
      WebSerial.println("Latitude: " + String(pBD->Latitude));
      WebSerial.println("Longitude: " + String(pBD->Longitude));
    }
#endif
    if (words[i].equals("webserver")) {
      WebSerial.print("local IP: ");
      WebSerial.println(WiFi.localIP());
      WebSerial.print("AP IP address: ");
      WebSerial.println(WiFi.softAPIP());
        int clientCount = WiFi.softAPgetStationNum();
        if (clientCount > 0) {
          logTo::logToAll("Clients connected: " + clientCount);
        } else {
          WebSerial.println("No clients connected");
=======
      logToAll("Latitude: " + String(pBD->Latitude));
      logToAll("Longitude: " + String(pBD->Longitude));
    }
#endif
    if (words[i].equals("webserver")) {
      logToAll("local IP: " + WiFi.localIP());
      logToAll("AP IP address: " + WiFi.softAPIP());
        int clientCount = WiFi.softAPgetStationNum();
        if (clientCount > 0) {
          logToAll("Clients connected: " + clientCount);
        } else {
          logToAll("No clients connected");
>>>>>>> 6b69be646e028764cc6b07b27ebd843c0deca33f
        }

    }
    if (words[i].equals("windrx")) {
<<<<<<< HEAD
      WebSerial.println("last wind time: " + String(time_since_last_wind_rx) + " avg wind time: " + String(avg_time_since_last_wind) + " ms");
      if (time_since_last_wind_rx > 0.0)
        WebSerial.println(String(1000.0/avg_time_since_last_wind) + " Hz (confirm timing 1000?)");
    }    
    if (words[i].equals("teleplot")) {
      compass.teleplot = !compass.teleplot;
      logTo::logToAll("teleplot: " + String(compass.teleplot));
=======
      logToAll("last wind time: " + String(time_since_last_wind_rx) + " avg wind time: " + String(avg_time_since_last_wind) + " ms");
      if (time_since_last_wind_rx > 0.0)
        logToAll(String(1000.0/avg_time_since_last_wind) + " Hz (confirm timing 1000?)");
    }    
    if (words[i].equals("teleplot")) {
      teleplot = !teleplot;
      logToAll("teleplot: " + teleplot?" on":" off");
>>>>>>> 6b69be646e028764cc6b07b27ebd843c0deca33f
      return;
    }
    if (words[i].equals("hostname")) {
      if (!words[++i].isEmpty()) {
        host = words[i];
        preferences.putString("hostname", host);
<<<<<<< HEAD
        logTo::logToAll("hostname set to " + host + "\n");
        logTo::logToAll("restart to change hostname\n");
        logTo::logToAll("preferences " + preferences.getString("hostname"));
      } else {
        logTo::logToAll("hostname: " + host);
=======
        logToAll("hostname set to " + host + "\n");
        logToAll("restart to change hostname\n");
        logToAll("preferences " + preferences.getString("hostname"));
      } else {
        logToAll("hostname: " + host);
>>>>>>> 6b69be646e028764cc6b07b27ebd843c0deca33f
      }
      return;
    }  
    if (words[i].equals("hall")) {
<<<<<<< HEAD
      logTo::logToAll("got true heading trigger, setting orientation mast: " + String(mastCompassDeg) + " boat: " + String(boatCompassDeg) + " delta: " + String(mastDelta));
      mastOrientation = 0;
      mastOrientation = mastDelta = readCompassDelta(); 
      logTo::logToAll("new orientation: " + String(mastDelta));
=======
      logToAll("got true heading trigger, setting orientation mast: " + String(mastCompassDeg) + " boat: " + String(boatCompassDeg) + " delta: " + String(mastDelta));
      mastOrientation = 0;
      mastOrientation = mastDelta = readCompassDelta(); 
      logToAll("new orientation: " + String(mastDelta));
>>>>>>> 6b69be646e028764cc6b07b27ebd843c0deca33f
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
<<<<<<< HEAD
      logTo::logToAll("frequency %d\n", frequency);
=======
      WebSerial.printf("frequency %d\n", frequency);
>>>>>>> 6b69be646e028764cc6b07b27ebd843c0deca33f
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
<<<<<<< HEAD
      logTo::logToAll("compass orientation %d\n",compassParams.orientation);
=======
      WebSerial.printf("compass orientation %d\n",compassParams.orientation);
>>>>>>> 6b69be646e028764cc6b07b27ebd843c0deca33f
      return;
    }
#endif
    if (words[i].startsWith("report")) {
      for (int i=0; i<SH2_MAX_SENSOR_ID; i++) {
<<<<<<< HEAD
        if (compass.numReports[i] > 0)
          logTo::logToAll("report 0x" + String(i,HEX) + "/" + String(i) + ": " + String(compass.numReports[i]));
      }
      logTo::logToAll("total reports " + String(compass.totalReports));
      return;
    }
    if (words[i].equals("teleplot")) {
      compass.teleplot = !compass.teleplot;
      logTo::logToAll("teleplot " + String(compass.teleplot));
      return;
    }  
=======
        if (numReports[i] > 0)
          WebSerial.printf("report 0x%x/%d: %d\n", i, i, numReports[i]);
      }
      WebSerial.printf("total reports %d\n", totalReports);
      return;
    }
    if (words[i].equals("teleplot")) {
      teleplot = !teleplot;
      WebSerial.printf("teleplot %s\n", teleplot ? "on" : "off");
      return;
    }
>>>>>>> 6b69be646e028764cc6b07b27ebd843c0deca33f
#if 0 // BNO  
    if (words[i].equals("rtype")) {
      if (!words[++i].isEmpty()) {
        reportType = (int)strtol(words[i].c_str(), NULL, 16);
        preferences.putInt("rtype", reportType);
<<<<<<< HEAD
        logTo::logToAll("compass report type set to 0x%x\n", reportType);
#ifdef BNO08X
        if (!bno08x.enableReport(reportType)) 
                logTo::logToAll("Could not enable local report 0x%x\n",reportType);
#endif
      } else {
        logTo::logToAll("compass report type is 0x%x\n",reportType);
=======
        WebSerial.printf("compass report type set to 0x%x\n", reportType);
#ifdef BNO08X
        if (!bno08x.enableReport(reportType)) 
                WebSerial.printf("Could not enable local report 0x%x\n",reportType);
#endif
      } else {
        WebSerial.printf("compass report type is 0x%x\n",reportType);
>>>>>>> 6b69be646e028764cc6b07b27ebd843c0deca33f
      }
      return;
    }
#endif
  }
  for (int i=0; i<wordCount; i++) words[i] = String();
  dataS = String();
}
<<<<<<< HEAD

=======
>>>>>>> 6b69be646e028764cc6b07b27ebd843c0deca33f
