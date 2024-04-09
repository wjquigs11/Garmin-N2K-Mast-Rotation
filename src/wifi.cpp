
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
#include <Arduino_JSON.h>
#include <ESPmDNS.h>
//#include <ArduinoJson.h>


// Search for parameter in HTTP POST request
const char* PARAM_INPUT_1 = "ssid";
const char* PARAM_INPUT_2 = "pass";
const char* PARAM_INPUT_3 = "ip";
const char* PARAM_INPUT_4 = "gateway";

struct wifiConfig {
  char hostname[64];
  char ssid[64];
  char pass[128];
  char ip[64];
  char gateway[64]; 
  int port;
};

wifiConfig wConfig;

/*Variables to save values from HTML form
String ssid;
String pass;
String ip;
String gateway;
*/

/* File paths to save input values permanently
const char* ssidPath = "/ssid.txt";
const char* passPath = "/pass.txt";
const char* ipPath = "/ip.txt";
const char* gatewayPath = "/gateway.txt";
*/
const char *configPath = "/wifiConf.json";

IPAddress localIP;

// Set your Gateway IP address
IPAddress localGateway;

IPAddress subnet(255, 255, 255, 0);

const char *hostname = "ESPWind";

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

/* Read File from SPIFFS
bool readConfig(fs::FS &fs, const char * path){
  Serial.printf("Reading file: %s\r\n", path);

  JsonDocument doc;
  File file = fs.open(path);
  if(!file || file.isDirectory()){
    Serial.println("- failed to open file for reading");
    return false;
  }
  DeserializationError error = deserializeJson(doc, file);
  if (error)
    Serial.println(F("Failed to read file, using default configuration"));

  // Copy values from the JsonDocument to the Config
  wConfig.port = doc["port"] | 80;
  strlcpy(wConfig.hostname,                  // <- destination
          doc["hostname"] | hostname,  // <- source
          sizeof(wConfig.hostname));         // <- destination's capacity
  strlcpy(wConfig.ssid, doc["ssid"], sizeof(wConfig.ssid));
  strlcpy(wConfig.pass, doc["pass"], sizeof(wConfig.pass));
  strlcpy(wConfig.ip, doc["ip"], sizeof(wConfig.ip));
  strlcpy(wConfig.gateway, doc["gateway"], sizeof(wConfig.gateway));
  
  file.close();   // File's destructor doesn't close the file
  return true;
}
*/
// Write file to SPIFFS
void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("- failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("- file written");
  } else {
    Serial.println("- write failed");
  }
}

// Initialize SPIFFS
void initSPIFFS() {
  if (!SPIFFS.begin()) {
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  Serial.println("SPIFFS mounted successfully");
  // Load values saved in SPIFFS
  if (readConfig(SPIFFS, configPath))
    Serial.println("OK read config from SPIFFS");
  Serial.println(wConfig.ssid);
  Serial.println(wConfig.pass);
  Serial.println(wConfig.ip);
  Serial.println(wConfig.gateway);
}

bool APmodeSwitch = false;

bool initWiFi() {
  if(wConfig.ssid=="") {
    Serial.println("Undefined SSID.");
    return false;
  }
  WiFi.setHostname(wConfig.hostname);
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
    delay(100);
    result = WiFi.begin(wConfig.ssid, wConfig.pass);
  }
  Serial.println(WiFi.localIP());
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
            strlcpy(wConfig.ssid, p->value().c_str(), sizeof(wConfig.ssid));
            Serial.print("SSID set to: ");
            Serial.println(wConfig.ssid);
            //writeFile(SPIFFS, ssidPath, ssid.c_str());
          }
          // HTTP POST pass value
          if (p->name() == PARAM_INPUT_2) {
            strlcpy(wConfig.pass, p->value().c_str(), sizeof(wConfig.pass));
            Serial.print("Password set to: ");
            Serial.println(wConfig.pass);
            // Write file to save value
            //writeFile(SPIFFS, passPath, pass.c_str());
          }
          // HTTP POST ip value
          if (p->name() == PARAM_INPUT_3) {
            strlcpy(wConfig.ip, p->value().c_str(), sizeof(wConfig.ip));
            Serial.print("IP Address set to: ");
            Serial.println(wConfig.ip);
            // Write file to save value
            //writeFile(SPIFFS, ipPath, ip.c_str());
          }
          // HTTP POST gateway value
          if (p->name() == PARAM_INPUT_4) {
            strlcpy(wConfig.gateway, p->value().c_str(), sizeof(wConfig.gateway));
            Serial.print("Gateway set to: ");
            Serial.println(wConfig.gateway);
            // Write file to save value
            //writeFile(SPIFFS, gatewayPath, gateway.c_str());
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
