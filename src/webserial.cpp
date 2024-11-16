#ifdef WINDBUS
#include "windparse.h"
#include "BoatData.h"
#include "wind-bus.h"

// object-oriented classes
#ifdef BNO08X
#include "BNO085Compass.h"
#endif
#include "logto.h"
#ifdef N2K
#include "n2k.h"
extern tNMEA2000 *n2kMain;
#endif

extern Preferences preferences;
char prbuf[PRBUF];
extern File consLog;

extern tBoatData *pBD;
extern tBoatData BoatData;

extern Stream *forward_stream;

// defs for wifi
void initWebSocket();
//void notifyClients(String);
extern AsyncWebServer server;
extern bool serverStarted;
extern char *hostname;
extern int WebTimerDelay;
extern AsyncEventSource events;
//extern JSONVar readings;
extern JsonDocument readings;
extern void setupWifi();
//extern String host;
extern void loopWifi();
void startWebServer();
void writeWiFi(int priority, String ssidNew, String passwdNew);

void httpInit(const char* serverName);
extern const char* serverName;

extern float mastDelta;

void mastHeading();
float readCompassDelta();
extern int mastAngle;

extern bool displayOnToggle;
/* bool teleplot=false;
extern int numReports[], totalReports;
*/

//extern elapsedMillis time_since_last_mastcomp_rx;

bool logTo::logToSerial = true;

void logTo::logToAll(String s) {
  if (s.endsWith("\n")) s.remove(s.length() - 1);
  String t = "[" + String(millis() / 1000) + "]: ";
  if (logToSerial) Serial.print(t + s);
  //consLog.println(s);
  if (serverStarted)
    WebSerial.println(t + s);
  s = t = String();
}

String logTo::commandList[logTo::ASIZE] = {"?", "format", "restart", "ls", "scan", "status", "readings", "mast", "lsap", "toggle",
  "gps", "webserver", "compass", "windrx", "espnow", "teleplot", "hostname", "rtype", "n2k", "wifi", "serial"};
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
#ifdef N2K
      WebSerial.println("           AWS (in): " + String(n2k::windSpeedKnots));
      WebSerial.println("           AWA (in): " + String(n2k::windAngleDegrees));
      WebSerial.println("       Mast Compass: " + String(n2k::mastIMUdeg));
      WebSerial.println("         Mast angle: " + String(n2k::mastDelta));
#endif
      WebSerial.flush();
    }

#ifdef N2K
    if (words[i].equals("n2k")) {
      WebSerial.println("  n2k main xmit/rcv: " + String(n2k::num_n2k_xmit) + "/" + String(n2k::num_n2k_recv));
      WebSerial.println("  n2k wind xmit/rcv: " + String(n2k::num_wind_xmit) + "/" + String(n2k::num_wind_recv));
      WebSerial.println("      n2k wind fail: " + String(n2k::num_wind_fail));
      WebSerial.println("       n2k wind fwd: " + String(n2k::num_wind_other));
      WebSerial.println("  n2k wind fwd fail: " + String(n2k::num_wind_other_fail));
      WebSerial.println("    n2k wind fwd ok: " + String(n2k::num_wind_other_ok));
      if (n2k::otherPGNindex > 0) {
        WebSerial.print("n2k other PGNs: ");
        for (int i=0; i<n2k::otherPGNindex; i++) {
          WebSerial.print(n2k::otherPGN[i]);
          if (i+1<n2k::otherPGNindex) WebSerial.print(", ");
        }
        WebSerial.println();
      }
    }
#endif

#if 0
    if (words[i].equals("format")) {
      SPIFFS.format();
      WebSerial.println("SPIFFS formatted");
    }
#endif

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
      i2cScan(Wire);
    }

    if (words[i].startsWith("readi")) {
      String jsonString;
      serializeJson(readings, jsonString);
      WebSerial.println(jsonString);
      jsonString = String();
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

    if (words[i].startsWith("disp")) {
      WebSerial.println("Display: " + String(displayOnToggle));
    }

    if (words[i].startsWith("webse")) {
      WebSerial.print("local IP: ");
      WebSerial.println(WiFi.localIP());
      WebSerial.print("AP IP address: ");
      WebSerial.println(WiFi.softAPIP());
        int clientCount = WiFi.softAPgetStationNum();
        if (clientCount > 0) {
          logTo::logToAll("Clients connected: " + clientCount);
        } else {
          WebSerial.println("No clients connected");
        }

    if (words[i].startsWith("seri")) {
      logTo::logToSerial = !logTo::logToSerial;
      WebSerial.println("logToSerial: " + String(logTo::logToSerial));
    }
    }

#ifdef N2K
    if (words[i].equals("windrx")) {
      WebSerial.println("last wind time: " + String(n2k::time_since_last_wind_rx) + " avg wind time: " + String(n2k::avg_time_since_last_wind) + " ms");
      if (n2k::time_since_last_wind_rx > 0.0)
        WebSerial.println(String(1000.0/n2k::avg_time_since_last_wind) + " Hz (confirm timing 1000?)");
    }    
#endif

#ifdef BNO08X
    if (words[i].equals("teleplot")) {
      compass.teleplot = !compass.teleplot;
      logTo::logToAll("teleplot: " + String(compass.teleplot));
      return;
    }
#endif

    if (words[i].startsWith("hostn")) {
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

#ifdef N2K
    if (words[i].equals("hall")) {
      logTo::logToAll("got true heading trigger, setting orientation mast: " + String(n2k::mastIMUdeg) + " boat: " + String(n2k::boatIMUdeg) + " delta: " + String(n2k::mastDelta));
      n2k::mastOrientation = 0;
      n2k::mastOrientation = n2k::mastDelta = readCompassDelta(); 
      logTo::logToAll("new orientation: " + String(n2k::mastDelta));
    return;
    }
    if (words[i].equals("imu")) {
      logTo::logToAll("mast IMU: " + String(n2k::mastIMUdeg));
      logTo::logToAll("boatIMU: " + String(n2k::boatIMUdeg));
      logTo::logToAll("mastAngle: " + String(n2k::mastAngle));
    return;
    }
#endif

  for (int i=0; i<wordCount; i++) words[i] = String();
  dataS = String();
  }
}
#endif
