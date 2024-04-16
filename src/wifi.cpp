
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
#include <Arduino_JSON.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <Adafruit_ADS1X15.h>

// Search for parameter in HTTP POST request
const char* PARAM_INPUT_1 = "ssid";
const char* PARAM_INPUT_2 = "pass";
const char* PARAM_INPUT_3 = "ip";
const char* PARAM_INPUT_4 = "gateway";

struct wifiConfig {
  String hostname;
  String ssid;
  String pass;
  char ip[64];
  char gateway[64]; 
  int port;
};

wifiConfig wConfig;

IPAddress localIP;

// Set your Gateway IP address
IPAddress localGateway;

IPAddress subnet(255, 255, 255, 0);

const char *hostname = "ESPWind";
// calibration; saved to preferences
int portRange=50, stbdRange=50; // NB BOTH are positive
extern Adafruit_ADS1015 ads;
extern int PotLo, PotHi;
int MagLo, MagHi; // ends of range corresponding to PotLo/PotHi and portRange/stbdRange
// however, they're just used as a calibration sanity check since they will change all the time when the boat is moving
extern int magOrientation; // we *should* start getting valid magOrientation as soon as BLE is connected
Preferences preferences;     

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create an Event Source on /events
//AsyncEventSource events("/events");

// create websocket object
AsyncWebSocket ws("/ws");

// Json Variable to Hold Sensor Readings
JSONVar readings;
//const char *readings;
//JsonArray readings;

// Timer variables
unsigned long lastTime = 0;
int WebTimerDelay = 500;

// Initialize SPIFFS
bool initSPIFFS() {
  if (!SPIFFS.begin()) {
    Serial.println("An error has occurred while mounting SPIFFS");
    return false;
  }
  Serial.println("SPIFFS mounted successfully");
  preferences.begin("wifi", false);                        
  wConfig.hostname = preferences.getString("hostname",hostname).c_str();  
  wConfig.ssid = preferences.getString("ssid").c_str();  
  wConfig.pass = preferences.getString("pass").c_str();
  Serial.println(wConfig.hostname);
  return true;
}

// Replaces HTML %placeholder% with stored values
String cal_processor(const String& var) {
  //Serial.println(var);
  if (var == "portRange")
    return String(preferences.getInt("portRange"));
  if (var == "stbdRange")
    return String(preferences.getInt("stbdRange"));
  if (var == "mastAngle")
    return String(ads.readADC_SingleEnded(0));
  return "not found";
}

bool APmodeSwitch = false;

bool initWifi() {
  if(wConfig.ssid=="") {
    Serial.println("Undefined SSID.");
    return false;
  }
  WiFi.setHostname(wConfig.hostname.c_str());
  WiFi.mode(WIFI_STA);
  if (wConfig.ip!="") {
    localIP.fromString(wConfig.ip);
    if (wConfig.gateway!="")
      localGateway.fromString(wConfig.gateway);
    if (!WiFi.config(localIP, localGateway, subnet)) {
      Serial.println("STA Failed to configure");
      return false;
    }
  }
  int result = WiFi.begin(wConfig.ssid, wConfig.pass);
  Serial.println("Connecting to WiFi...");
  unsigned long currentMillis = millis();
  lastTime = currentMillis;
  while(WiFi.status() != WL_CONNECTED) {
    currentMillis = millis();
    if (currentMillis - lastTime >= 5000) {
      Serial.print("Failed to connect, status: ");
      Serial.println(result);
      lastTime = currentMillis;
      return false;
    }
    delay(1000);
    result = WiFi.begin(wConfig.ssid, wConfig.pass);
  }
  Serial.println(result);
  Serial.println(WiFi.localIP());

    // start serving from SPIFFS
    server.serveStatic("/", SPIFFS, "/");

    server.on("/calibrate", HTTP_GET, [](AsyncWebServerRequest *request) {
          request->send(SPIFFS, "/calibrate.html", "text/html", cal_processor);
    });

    // POST on calibrate means we've gotten rotation range parameters
    server.on("/calibrate", HTTP_POST, [](AsyncWebServerRequest *request) {
      int params = request->params();
      for(int i=0;i<params;i++) {
        AsyncWebParameter* p = request->getParam(i);
        if(p->isPost()) {
          // HTTP POST ssid value
          if (p->name() == "portRange") {
            portRange = atoi(p->value().c_str());
            Serial.print("portRange: ");
            Serial.println(portRange);
            //preferences?
          }
          if (p->name() == "stbdRange") {
            stbdRange = atoi(p->value().c_str());
            Serial.print("stbdRange: ");
            Serial.println(stbdRange);
            preferences.putInt("stbdRange", stbdRange);
          }
        } // isPost
      } // for params
      request->send(SPIFFS, "/calibrate2.html", "text/html"); // go to 2nd page
    });

    // POST on calibrate2 means mast is all the way to port
    server.on("/calibrate2", HTTP_POST, [](AsyncWebServerRequest *request) {
      // read sensors at port end of range (Honeywell and/or magnetic)
      PotLo = ads.readADC_SingleEnded(0);
      request->send(SPIFFS, "/calibrate3.html", "text/html");
    });

    // POST on calibrate3 means mast is all the way to starboard
    server.on("/calibrate3", HTTP_POST, [](AsyncWebServerRequest *request) {
      // read sensors at port end of range (Honeywell and/or magnetic)
      PotHi = ads.readADC_SingleEnded(0);
      request->send(SPIFFS, "/calibrate4.html", "text/html", cal_processor);
    });

    // POST on calibrate4 means user has pressed OK; save values
    // (cancel will redirect to /index.html)
    server.on("/calibrate4", HTTP_POST, [](AsyncWebServerRequest *request) {
      preferences.putInt("portRange", portRange);
      preferences.putInt("stbdRange", stbdRange);
      // what now? If you put a dial in calibrate4, maybe just refresh, but delete the buttons
      // or put the dial on index.html with a Calibrate button
      //request->send(SPIFFS, "/calibrate4.html", "text/html", cal_processor);
    });

    server.begin(); // start dishing up those web pages

  return true;
}

void APmode() {
    Serial.print(APmodeSwitch);
    Serial.print(" Setting Access Point: ");
    Serial.println(hostname);
    // NULL sets an open Access Point
    if (!WiFi.softAP(hostname, NULL)) {
      Serial.println("AP failed");
    }

    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP); 

    // Web Server Root URL, serve wifimanager.html
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/wifimanager.html", "text/html");
    });
    
    server.serveStatic("/", SPIFFS, "/");
    
    server.on("/", HTTP_POST, [](AsyncWebServerRequest *request) {
      int params = request->params();
      for(int i=0;i<params;i++){
        AsyncWebParameter* p = request->getParam(i);
        if(p->isPost()){
          // HTTP POST ssid value
          if (p->name() == PARAM_INPUT_1) {
            wConfig.ssid = p->value();
            preferences.putString("ssid", wConfig.ssid);
            Serial.print("SSID set to: ");
            Serial.println(wConfig.ssid);
          }
          // HTTP POST pass value
          if (p->name() == PARAM_INPUT_2) {
            wConfig.pass = p->value();
            preferences.putString("pass", wConfig.pass);
            Serial.print("Password set to: ");
            Serial.println(wConfig.pass);
          }
          // HTTP POST ip value
          if (p->name() == PARAM_INPUT_3) {
            Serial.print("IP Address set to: ");
            Serial.println(wConfig.ip);
          }
          // HTTP POST gateway value
          if (p->name() == PARAM_INPUT_4) {
            Serial.print("Gateway set to: ");
            Serial.println(wConfig.gateway);
          }
          //Serial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
        }
      }
      char buf[128];
      sprintf(buf, "Done. ESP will restart, connect to your router and go to IP address: %s", wConfig.ip);
      request->send(200, "text/plain", buf);
      delay(5000);
      ESP.restart();
    });
    
    server.begin();
}

// do the websockets stuff

void notifyClients(String sensorReadings) {
  ws.textAll(sensorReadings);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      String jsonString = JSON.stringify(readings);
      Serial.println("sending readings from handleWebSocketMessage");
      notifyClients(jsonString);
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}
/*
void loop() {
  if ((millis() - lastTime) > WebTimerDelay) {
    if (APmodeSwitch) {
      String sensorReadings = getSensorReadings();
      Serial.println(sensorReadings);
      notifyClients(sensorReadings);
    } else {
      // we're still in AP mode
      Serial.print(".");
    }
    lastTime = millis();
  }
  ws.cleanupClients();
}*/
