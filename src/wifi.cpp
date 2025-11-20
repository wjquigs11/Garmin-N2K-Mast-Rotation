#ifdef WIFI
/*
Start wifi. If there are credentials saved in a file, use them.
If not, start a captive portal (using DNS).
*/

#include "include.h"

const char* wifiPath = "/wifi.txt";
String ssid = "";
String password = "";
String ip;
String gateway;

// for captive portal
DNSServer dnsServer;

// Search for parameter in HTTP POST request
const char* PARAM_HOSTNAME = "hostname";
const char* PARAM_SSID = "ssid";
const char* PARAM_PASSWORD = "pass";
const char* PARAM_IP = "ip";
const char* PARAM_GATEWAY = "gateway";

// Read WiFi credentials from wifi.txt file
bool readWiFiCredentials() {
  File file = SPIFFS.open(wifiPath);
  if(!file || file.isDirectory()) {
    log::toAll("Failed to open wifi.txt for reading");
    return false;
  }
  
  // Read the first line which contains SSID:password[:ip][:gateway]
  String fileContent = "";
  if(file.available()) {
    fileContent = file.readStringUntil('\n');
  }
  file.close();
  log::toAll(fileContent);
  if(fileContent.length() > 0) {
    // Parse the content by splitting at colons
    int firstColonPos = fileContent.indexOf(':');
    if(firstColonPos > 0) {
      ssid = fileContent.substring(0, firstColonPos);
      // Get the rest of the string after the first colon
      String remaining = fileContent.substring(firstColonPos + 1);
      // Check if there's another colon for IP address
      int secondColonPos = remaining.indexOf(':');
      if(secondColonPos > 0) {
        password = remaining.substring(0, secondColonPos);
        remaining = remaining.substring(secondColonPos + 1);
        // Check if there's another colon for Gateway
        int thirdColonPos = remaining.indexOf(':');
        if(thirdColonPos > 0) {
          ip = remaining.substring(0, thirdColonPos);
          gateway = remaining.substring(thirdColonPos + 1);
          gateway.trim();
        } else {
          // Only IP is present, no gateway
          ip = remaining;
          ip.trim();
        }
      } else {
        // Only SSID and password
        password = remaining;
        password.trim(); // Trim any whitespace or special characters
      }
      log::toAll("Read WiFi credentials with IP and Gateway - SSID: " + ssid +
                    ", Password length: " + String(password.length()) +
                    ", IP: " + ip + ", Gateway: " + gateway);
      return true;
    }
  }
  log::toAll("No valid WiFi credentials found in wifi.txt");
  return false;
}

void setUpDNSServer(DNSServer &dnsServer, const IPAddress &localIP) {
  // Set the TTL for DNS response and start the DNS server
  log::toAll("starting DNS server for captive portal");
  dnsServer.setTTL(3600);
  dnsServer.start(53, "*", localIP);
  // we modify loop() delay here; not sure if this is necessary
  loopDelay = DNS_INTERVAL;
}

void startPortal() {
    // Connect to Wi-Fi network with SSID and password
    log::toAll("Setting AP (Access Point)");
    // NULL sets an open Access Point
    WiFi.softAP("ESP-SETUP", NULL);

    IPAddress IP = WiFi.softAPIP();
    log::toAll("AP IP address: " + IP.toString());

    setUpDNSServer(dnsServer, IP);
  String localIPURL = "http://" + IP.toString();

  // https://github.com/CDFER/Captive-Portal-ESP32
  //======================== Webserver ========================
  // WARNING IOS (and maybe macos) WILL NOT POP UP IF IT CONTAINS THE WORD "Success" https://www.esp8266.com/viewtopic.php?f=34&t=4398
  // SAFARI (IOS) IS STUPID, G-ZIPPED FILES CAN'T END IN .GZ https://github.com/homieiot/homie-esp8266/issues/476 this is fixed by the webserver serve static function.
  // SAFARI (IOS) there is a 128KB limit to the size of the HTML. The HTML can reference external resources/images that bring the total over 128KB
  // SAFARI (IOS) popup browser has some severe limitations (javascript disabled, cookies disabled)

  // Required
  server.on("/connecttest.txt", [](AsyncWebServerRequest *request) { request->redirect("http://logout.net"); });  // windows 11 captive portal workaround
  server.on("/wpad.dat", [](AsyncWebServerRequest *request) { request->send(404); });                // Honestly don't understand what this is but a 404 stops win 10 keep calling this repeatedly and panicking the esp32 :)

  // Background responses: Probably not all are Required, but some are. Others might speed things up?
  // A Tier (commonly used by modern systems)
  server.on("/generate_204", [localIPURL](AsyncWebServerRequest *request) { request->redirect(localIPURL); });       // android captive portal redirect
  server.on("/redirect", [localIPURL](AsyncWebServerRequest *request) { request->redirect(localIPURL); });           // microsoft redirect
  server.on("/hotspot-detect.html", [localIPURL](AsyncWebServerRequest *request) { request->redirect(localIPURL); });  // apple call home
  server.on("/canonical.html", [localIPURL](AsyncWebServerRequest *request) { request->redirect(localIPURL); });     // firefox captive portal call home
  server.on("/success.txt", [](AsyncWebServerRequest *request) { request->send(200); });                   // firefox captive portal call home
  server.on("/ncsi.txt", [localIPURL](AsyncWebServerRequest *request) { request->redirect(localIPURL); });           // windows call home

  // B Tier (uncommon)
  //  server.on("/chrome-variations/seed",[](AsyncWebServerRequest *request){request->send(200);}); //chrome captive portal call home
  //  server.on("/service/update2/json",[](AsyncWebServerRequest *request){request->send(200);}); //firefox?
  //  server.on("/chat",[](AsyncWebServerRequest *request){request->send(404);}); //No stop asking Whatsapp, there is no internet connection
  //  server.on("/startpage",[](AsyncWebServerRequest *request){request->redirect(localIPURL);});

  // return 404 to webpage icon
  server.on("/favicon.ico", [](AsyncWebServerRequest *request) { request->send(404); });  // webpage icon

  // the catch all
  server.onNotFound([localIPURL](AsyncWebServerRequest *request) {
    request->redirect(localIPURL);
    log::toAll("onnotfound " + request->host() + " " + request->url() + " sent redirect to " + localIPURL);
  });

  // Web Server Root URL: serve wifimanager.html from captive portal
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/wifimanager.html", "text/html");
  });

  server.on("/", HTTP_POST, [](AsyncWebServerRequest *request) {
      int params = request->params();
      for (int i=0; i<params; i++) {
        const AsyncWebParameter* p = request->getParam(i);
        if(p->isPost()) {
          if (p->name() == PARAM_HOSTNAME) {
            // hostname is special; goes into preferences instead of file
            host = p->value().c_str();
            preferences.putString("hostname", host);
            log::toAll("host set to: " + host);
          }
          if (p->name() == PARAM_SSID) {
            ssid = p->value().c_str();
            log::toAll("SSID set to: " + ssid);
          }
          if (p->name() == PARAM_PASSWORD) {
            password = p->value().c_str();
            log::toAll("Password set to: " + password);
          }
          if (p->name() == PARAM_IP) {
            ip = p->value().c_str();
            log::toAll("IP Address set to: " + ip);
          }
          if (p->name() == PARAM_GATEWAY) {
            gateway = p->value().c_str();
            log::toAll("Gateway set to: " + gateway);
          }
          log::toAll("POST[" + p->name() + "]: " + p->value());
        }

        // Save the WiFi credentials to the file
        File file = SPIFFS.open(wifiPath, FILE_WRITE);
        if(!file) {
          log::toAll("Failed to open wifi.txt for writing");
        } else {
          // Format: SSID:password:ip:gateway
          String fileContent = ssid + ":" + password;
          if(ip.length() > 0) {
            fileContent += ":" + ip;
            if(gateway.length() > 0) {
              fileContent += ":" + gateway;
            }
          }
          file.println(fileContent);
          file.close();
          log::toAll("WiFi credentials saved to file");
        }
      } // for i params
    request->send(200, "text/plain", "Done. ESP will restart, connect to your router and go to IP address: " + ip);
    delay(3000);
    ESP.restart();
  }); // end lambda for "/"

  server.begin();
}

bool setupWifi() {
  log::toAll("Starting WiFi...");
  // Try to read credentials from file first
  bool hasCredentials = readWiFiCredentials();
  if (!hasCredentials) {
    log::toAll("no credentials, starting captive portal");
    startPortal();
  } else {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
        log::toAll("WiFi Failed!");
        startPortal();
    } else {
        // Successfully connected to WiFi
        log::toAll("ESP IP Address: http://" + WiFi.localIP().toString());
        
#ifdef NTP
        // Configure NTP server and timezone
        log::toAll("Setting up time synchronization...");
        setupTime();
        
        // Wait for time synchronization (timeout after 10 seconds)
        waitForTimeSync(10);
        
        // If time sync was successful, log the current time
        if (isNtpSyncSuccessful()) {
            log::toAll("Current time: " + getFormattedTime());
        } else {
            log::toAll("Failed to sync time with NTP servers. Will use browser time if available.");
        }
#endif        
        server.begin();
        return true;
    }
  }
  return false;
}

void resetWifi() {
  log::toAll("Resetting WiFi settings...");
  WiFi.disconnect(true);
  if (SPIFFS.exists(wifiPath))
    if (SPIFFS.remove(wifiPath))
      log::toAll("removed wifi params");
  delay(3000);
  ESP.restart();
}
#endif // ifdef WIFI