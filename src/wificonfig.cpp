#include <DNSServer.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include "ESPAsyncWebServer.h"
#include <Preferences.h>
#include "SPIFFS.h"
#include <Arduino_JSON.h>

Preferences preferences;
DNSServer dnsServer;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
JSONVar readings;

String ssid;
String password;
String ip;
String gateway;
const char *hostname = "ESPWind";

bool is_setup_done = false;
bool valid_ssid_received = false;
bool valid_password_received = false;
bool wifi_timeout = false;

IPAddress localIP;
IPAddress localGateway;
IPAddress subnet(255, 255, 255, 0);

// Timer variables
unsigned long lastTime = 0;
int WebTimerDelay = 500;

void StartCaptivePortal();

// define index.html for use when no AP has been defined (captive portal)
// we ask for ssid, pass and do HTML GET to /get to "post"
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>Captive Portal Demo</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  </head><body>
  <h3>Captive Portal Demo</h3>
  <br><br>
  <form action="/get">
    <br>
    SSID: <input type="text" name="ssid">
    <br>
    Password: <input type="text" name="password">
    <input type="submit" value="Submit">
  </form>
</body></html>)rawliteral";

class CaptiveRequestHandler : public AsyncWebHandler {
  public:
    CaptiveRequestHandler() {}
    virtual ~CaptiveRequestHandler() {}

    bool canHandle(AsyncWebServerRequest *request) {
      //request->addInterestingHeader("ANY");
      return true;
    }

    void handleRequest(AsyncWebServerRequest *request) {
      request->send_P(200, "text/html", index_html);
    }
};

// configuration server; will change to be served from SPIFFS after config done
void setupConfigServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/html", index_html);
    Serial.println("Client Connected");
  });

  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest * request) {
    String inputMessage;
    String inputParam;

    if (request->hasParam("ssid")) {
      inputMessage = request->getParam("ssid")->value();
      inputParam = "ssid";
      ssid = inputMessage;
      Serial.println(inputMessage);
      valid_ssid_received = true;
    }

    if (request->hasParam("password")) {
      inputMessage = request->getParam("password")->value();
      inputParam = "password";
      password = inputMessage;
      Serial.println(inputMessage);
      valid_password_received = true;
    }
    request->send(200, "text/html", "The values entered by you have been successfully sent to the device. It will now attempt WiFi connection");
  });
}


void WiFiSoftAPSetup()
{
  WiFi.mode(WIFI_AP);
  WiFi.softAP(hostname);
  Serial.print("AP IP address: "); Serial.println(WiFi.softAPIP());
}


//bool WiFiStationSetup(String rec_ssid, String rec_password)
bool WiFiStationSetup()
{
  wifi_timeout = false;
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(hostname);
  char ssid_arr[20];
  char password_arr[20];
  //rec_ssid.toCharArray(ssid_arr, rec_ssid.length() + 1);
  //rec_password.toCharArray(password_arr, rec_password.length() + 1);
  ssid.toCharArray(ssid_arr, ssid.length() + 1);
  password.toCharArray(password_arr, password.length() + 1);
  Serial.print("Received SSID: "); Serial.println(ssid_arr); Serial.print("And password: "); Serial.println(password_arr);
  if (ip!="") {
    localIP.fromString(ip);
    if (gateway!="")
      localGateway.fromString(gateway);
    if (!WiFi.config(localIP, localGateway, subnet)) {
      Serial.println("STA Failed to configure");
      return false;
    }
  }
  int result = WiFi.begin(ssid_arr, password_arr);

  uint32_t t1 = millis();
  // try to connect to wifi. If not, start captive portal
  while (WiFi.status() != WL_CONNECTED) {
    delay(2000);
    Serial.print(".");
    if (millis() - t1 > 10000) {
      Serial.println();
      Serial.println("Timeout connecting to WiFi. The SSID and Password seem incorrect.");
      valid_ssid_received = false;
      valid_password_received = false;
      is_setup_done = false;
      preferences.putBool("is_setup_done", is_setup_done);

      StartCaptivePortal();
      wifi_timeout = true;
      return false;
    }
  }
  if (!wifi_timeout) {
    is_setup_done = true;
    //Serial.println("");  Serial.print("WiFi connected to: "); Serial.println(rec_ssid);
    Serial.println("");  Serial.print("WiFi connected to: "); Serial.println(ssid);
    Serial.print("IP address: ");  Serial.println(WiFi.localIP());
    preferences.putBool("is_setup_done", is_setup_done);
    //preferences.putString("ssid", rec_ssid);
    //preferences.putString("password", rec_password);
    preferences.putString("ssid", ssid);
    preferences.putString("password", password);

    // start serving from SPIFFS
    server.serveStatic("/", SPIFFS, "/");
  }
  return true;
}

// couldn't connect to wifi
void StartCaptivePortal() {
  Serial.println("Setting up AP Mode");
  WiFiSoftAPSetup();
  Serial.println("Setting up Async WebServer");
  setupConfigServer();
  Serial.println("Starting DNS Server");
  dnsServer.start(53, "*", WiFi.softAPIP());
  server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER);//only when requested from AP
  server.begin();
  dnsServer.processNextRequest();
}

void getWifiPrefs() {
  preferences.begin("wifi-pref", false);
  is_setup_done = preferences.getBool("is_setup_done", false);
  ssid = preferences.getString("ssid", "Sample_SSID");
  password = preferences.getString("password", "abcdefgh");
  // TBD ip = preferences.getString();
}


void setupWifi() {

  if (!is_setup_done)
  {
    StartCaptivePortal();
  }
  else
  {
    Serial.println("Using saved SSID and Password to attempt WiFi Connection!");
    Serial.print("Saved SSID is ");Serial.println(ssid);
    Serial.print("Saved Password is ");Serial.println(password);
    //WiFiStationSetup(ssid, password);
    WiFiStationSetup();
  }

  while (!is_setup_done)
  {
    dnsServer.processNextRequest();
    delay(10);
    if (valid_ssid_received && valid_password_received)
    {
      Serial.println("Attempting WiFi Connection!");
      //WiFiStationSetup(ssid, password);
      WiFiStationSetup();
    }
  }

  Serial.println("All Done!");

  //Your Additional Setup code
}

/*
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
    // TBD: change index.html to wifimanager.html
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/wifimanager.html", "text/html");
    });
    
    server.serveStatic("/", SPIFFS, "/");
    
    //server.begin();
}
*/

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