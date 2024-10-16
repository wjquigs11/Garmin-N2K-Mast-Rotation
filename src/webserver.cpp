#ifdef WINDBUS
#include "windparse.h"
#include "compass.h"
#include "BoatData.h"
#include "wind-bus.h"

// object-oriented classes
#ifdef BNO08X
#include "BNO085Compass.h"
#endif
#include "logto.h"
#ifdef N2K
#include "n2k.h"
#endif

Preferences preferences;     

int compassFrequency;

// Create AsyncWebServer object on port 80
#define HTTP_PORT 80
AsyncWebServer server(HTTP_PORT);
AsyncEventSource events("/events");
//AsyncWebSocket ws("/ws");

// Json Variable to Hold Sensor Readings
//JSONVar readings;
JsonDocument readings;

// Timer variables
unsigned long lastTime = 0;
int WebTimerDelay = 500;

extern bool displayOnToggle, demoModeToggle;
//extern Adafruit_SSD1306 display;
const char* PARAM_INPUT_1 = "output";
const char* PARAM_INPUT_2 = "state";

//void demoInit();
int compassDifference(int angle1, int angle2);

// Get Sensor Readings and return JSON object
String getSensorReadings() {
#ifdef N2K
  readings["rotateout"] = n2k::rotateout;
  readings["windSpeed"] = n2k::windSpeedKnots;
  readings["windAngle"] = n2k::windAngleDegrees;
#endif
  readings["mastHeading"] = 0;
  readings["mastDelta"] = 0;
#ifdef BNO08X
  if (compass.OnToggle) {
    if (n2k::mastIMUdeg >= 0) { // set to -1 if mast compass times out
      readings["mastHeading"] = n2k::mastIMUdeg+n2k::mastOrientation;
      readings["mastDelta"] = mastAngle[1];
    }
    readings["boatHeading"] = n2k::boatIMUdeg;
    readings["boatTrue"] = BoatData.TrueHeading;
    //readings["boatCalStatus"] = String(boatCalStatus);
  }
#endif

  String jsonString;
  serializeJson(readings,jsonString); // returns an int?
  //JSON.stringify(readings);
  //logTo::logToAll(jsonString);
  return jsonString;
}

/*
  There's a placeholder in the html file %BUTTONPLACEHOLDER%
  When the page renders the "processor" function I define below will get called to replace the placeholder(s)
  with html generated and placed in the string(s) below
  The CSS in the HTML file changes the appearance of the slider based on whether the checkbox shows as "checked" or not
*/
String settingsString(100);
String settings_processor(const String& var) {
  logTo::logToAll("settings processor var: " + var);
  if (var == "BUTTONPLACEHOLDER") {
    settingsString = "<h4>Display</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"display\" ";
    if (displayOnToggle) settingsString += "checked";
    settingsString += "><span class=\"slider\"></span></label>";
    settingsString += "<h4>Mast Compass</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"compass\" ";
#ifdef BNO08X
    if (compass.OnToggle) settingsString += "checked"; 
#endif
    settingsString += "><span class=\"slider\"></span></label>";
    return settingsString;
  }
  if (var == "webtimerdelay") return (settingsString = String(WebTimerDelay));
#ifdef N2K
  if (var == "orientation") return (settingsString = String(n2k::mastOrientation));
  if (var == "boatorient") return (settingsString = String(n2k::boatOrientation));
#endif
  if (var == "frequency") return (settingsString = String(compassFrequency));
  if (var == "controlMAC") return (settingsString = String(WiFi.macAddress()));
  if (var == "variation") return (settingsString = String(BoatData.Variation));
  return (settingsString = String("settings processor: placeholder not found " + var));
}

String demo_processor(const String& var) {
  logTo::logToAll("demo processor var: " + var);
  if (var == "BUTTONPLACEHOLDER") {
    String buttons = "";
    buttons += "<h4>Demo Mode</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"demo\" ";
    if (demoModeToggle) buttons += "checked";
    buttons += "><span class=\"slider\"></span></label>";
    return buttons;
  }
  return String("demo processor: placeholder not found " + var);
}

void toggleCheckbox(const char* id) {
  logTo::logToAll("toggleCheckbox id: " + String(id));
  if (strcmp(id, "display") == 0) {
    displayOnToggle = !displayOnToggle;
    logTo::logToAll("setting display to " + String(displayOnToggle));
  }
#ifdef BNO08X
  if (strcmp(id, "compass") == 0) {
    logTo::logToAll("setting compass to " + String(compass.OnToggle));
    compass.OnToggle = !compass.OnToggle;
  }
#endif
}

// Replaces HTML %placeholder% with stored values
String cal_processor(const String& var) {
  //Serial.printf("cal processor var: %s", var.c_str());
  if (var == "portRange") {
    return String(preferences.getInt("portRange"));
  }
  if (var == "stbdRange") {
    return String(preferences.getInt("stbdRange"));
  }
  return String("cal processor: placeholder not found " + var);
}

#if 0
void compassPing() {  // ping mast compass needs work for use case where mast compass isn't connected to external AP
  if(WiFi.status()== WL_CONNECTED) {    
    httpC.begin(mastCompass.c_str());
    int httpResponseCode = httpC.GET();
    if (httpResponseCode>0) {
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
      String payload = httpC.getString();
      Serial.println(payload);
    } else {
      Serial.print("HTTP GET Error code: ");
      Serial.println(httpResponseCode);
    }
    httpC.end();
    }
#endif

void startWebServer() {
  logTo::logToAll("starting web server");
  preferences.begin("ESPwind", false);
  displayOnToggle = (preferences.getString("displayOnTog", "true") == "true") ? true : false;
  logTo::logToAll("display = " + String(displayOnToggle));
#ifdef BNO08X
  compass.OnToggle = (preferences.getString("compass.OnTog", "false") == "true") ? true : false;
  logTo::logToAll("compass = " + String(compass.OnToggle));
  readings["compass"] = (compass.OnToggle ? 1 : 0);
#endif
  //demoModeToggle = (preferences.getString("demoModeTog", "false") == "true") ? true : false;
  logTo::logToAll("demo = " + String(demoModeToggle));
  WebTimerDelay = preferences.getInt("WebTimerDelay", 500);
  //mastOrientation = preferences.getInt("mastOrientation", 0);
#ifdef N2K
  n2k::boatOrientation = preferences.getInt("boatOrientation", 0);
#endif
  compassFrequency = preferences.getInt("compassFreq", 50);
  logTo::logToAll("compassFrequency = " + String(compassFrequency));
  BoatData.Variation = preferences.getFloat("variation", VARIATION);

  if (!MDNS.begin(host.c_str()) ) {
    logTo::logToAll("Error starting MDNS responder.");
  } else
      logTo::logToAll("MDNS started " + host);

  // Add service to MDNS-SD
  if (!MDNS.addService("http", "tcp", HTTP_PORT)) {
    logTo::logToAll("MDNS add service failed");
  }
  
  // SERVER INIT
  events.onConnect([](AsyncEventSourceClient * client) {
    client->send("hello!", NULL, millis(), 1000);
  });

  server.addHandler(&events);

  // start serving from SPIFFS
  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");
  server.serveStatic("/", SPIFFS, "/");

  // here's another bit of ugliness, until I figure out how to dynamically show/hide gauges in JS
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    logTo::logToAll("index.html");
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.on("/heap", HTTP_GET, [](AsyncWebServerRequest * request) {
    logTo::logToAll("heap.html");
    request->send(200, "text/plain", String(ESP.getFreeHeap()));
  });
  
  server.on("/demo", HTTP_GET, [](AsyncWebServerRequest *request) {
    logTo::logToAll("demo.html");
    request->send(SPIFFS, "/demo.html", "text/html", false, demo_processor);
  });

  // Request the latest sensor readings
  /* As the rest of the code runs, receiving updates like wind speed and boat heading, 
     it updates a JSON array called "readings".
     The index of each array element represents a variable passed to javascript on the web page
     Any page that uses script.js and has an element whose "span id" is the same as one of the readings elements
     will have that element's value replaced by the latest data, if that reading value is in the array
     For example, windparse.cpp does this: readings["windSpeed"] = String(windSpeedKnots);
     So any page that needs windSpeed can create an element with that label and get the value 
  */
  server.on("/readings", HTTP_GET, [](AsyncWebServerRequest *request) {
    //logTo::logToAll("readings");
    String json;
    //json.reserve(512);
    json = getSensorReadings();
    //logTo::logToAll("sending readings " + String(json.length()));
    request->send(200, "application/json", json);
    //logTo::logToAll("readings sent");
    json = String();
  });

  server.on("/host", HTTP_GET, [](AsyncWebServerRequest *request) {
    String buf = "host: " + host + ", webtimerdelay: " + String(WebTimerDelay);
    logTo::logToAll(buf);
    request->send(200, "text/plain", buf.c_str());
    buf = String();
  });

  server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *request) {
    logTo::logToAll("settings.html");
    request->send(SPIFFS, "/settings.html", "text/html", false, settings_processor);
  });
#ifdef BNO08X
  server.on("/compass", HTTP_GET, [](AsyncWebServerRequest *request) {
    logTo::logToAll("compass.html");
    request->send(SPIFFS, "/compass.html", "text/html");
  });
#endif
  static int oldMastOrientation;
#if 0
  server.on("/mastcompass", HTTP_GET, [](AsyncWebServerRequest *request) {
    logTo::logToAll("mastcompass.html");
    String inputMessage1;
    String inputMessage2;
    oldMastOrientation = n2k::mastOrientation;
    n2k::mastOrientation = 0;
    // GET input1 value on <ESP_IP>/params?output=<inputMessage1>&state=<inputMessage2>
    if (request->hasParam("confirm")) {
      inputMessage1 = request->getParam("confirm")->value();
      logTo::logToAll("/mastcompass: " + inputMessage1);
      n2k::mastOrientation = compassDifference(n2k::boatIMUdeg, n2k::mastIMUdeg+n2k::mastOrientation);
    }
    if (request->hasParam("cancel")) {
      inputMessage1 = request->getParam("cancel")->value();
      logTo::logToAll("/mastcompass: " + inputMessage1);
      n2k::mastOrientation = oldMastOrientation;
    }
    request->send(SPIFFS, "/mastcompass.html", "text/html");
  });
#endif

  // Send a GET request to <ESP_IP>/params?output=<inputMessage1>&state=<inputMessage2>
  server.on("/params", HTTP_GET, [] (AsyncWebServerRequest *request) {
    //logTo::logToAll("params.html");
    String inputMessage1;
    String inputMessage2;
    // GET input1 value on <ESP_IP>/params?output=<inputMessage1>&state=<inputMessage2>
    if (request->hasParam(PARAM_INPUT_1) && request->hasParam(PARAM_INPUT_2)) {
      inputMessage1 = request->getParam(PARAM_INPUT_1)->value();
      inputMessage2 = request->getParam(PARAM_INPUT_2)->value();
      //digitalWrite(inputMessage1.toInt(), inputMessage2.toInt());
      //("/params got %s %s", inputMessage1, inputMessage2);
      logTo::logToAll("/params: " + inputMessage1 + " " + inputMessage2);
      if (inputMessage1 == "display") {
        if (inputMessage2 == "off") {
          logTo::logToAll("display off");
          displayOnToggle = false;
//#ifdef DISPLAYON
//          display->clearDisplay();
//#endif
        } else {
          logTo::logToAll("display on");
          displayOnToggle = true;
        }
        preferences.putString("displayOnTog", displayOnToggle ? "true" : "false");
      }
#ifdef BNO08X
      if (inputMessage1 == "compass") {
        if (inputMessage2 == "off") {
          logTo::logToAll("compass off");
          compass.OnToggle = false;
          readings["compass"] = 0;
        } else {
          logTo::logToAll("compass on");
          compass.OnToggle = true;
          readings["compass"] = 1;
        }
        preferences.putString("compass.OnTog", compass.OnToggle ? "true" : "false");
        //sendMastControl();
      }
#endif
      if (inputMessage1 == "demo") {
        if (inputMessage2 == "off") {
          logTo::logToAll("demo off");
          demoModeToggle = false;
        } else {
          logTo::logToAll("demo on");
          demoModeToggle = true;
          //demoInit();
        }
        preferences.putString("demoModeTog", demoModeToggle ? "true" : "false");
      }
    }
    else {
      inputMessage1 = "No message sent";
      inputMessage2 = "No message sent";
    }
    request->send(200, "text/plain", "OK");
    inputMessage1 = String();
    inputMessage2 = String();
  });

  // POST on params
  server.on("/params", HTTP_POST, [](AsyncWebServerRequest *request) {
    int params = request->params();
    for(int i=0;i<params;i++) {
      const AsyncWebParameter* p = request->getParam(i);
      if(p->isPost()) {
        // HTTP POST ssid value
        logTo::logToAll("params POST " + p->name() + " " + p->value());
        if (p->name() == "webtimerdelay") {
          WebTimerDelay = atoi(p->value().c_str());
          preferences.putInt("WebTimerDelay", WebTimerDelay);
        }
#ifdef N2K
        if (p->name() == "orientation") {
          n2k::mastOrientation = atoi(p->value().c_str());
          preferences.putInt("mastOrientation", n2k::mastOrientation);
        }
        if (p->name() == "boatorient") {
          n2k::boatOrientation = atoi(p->value().c_str());
          preferences.putInt("boatOrientation", n2k::boatOrientation);
        }
#endif
        if (p->name() == "frequency") {
          compassFrequency = atoi(p->value().c_str());
          preferences.putInt("compassFreq", compassFrequency);
        }
        if (p->name() == "variation") {
          BoatData.Variation = atof(p->value().c_str());
          preferences.putFloat("variation", BoatData.Variation);
        }
      } // isPost
    } // for params
    //request->send(SPIFFS, "/index.html", "text/html");
    request->redirect("/");  // Redirect to root/index
  });

  server.onNotFound([](AsyncWebServerRequest * request) {
    Serial.print(F("NOT_FOUND: "));
    if (request->method() == HTTP_GET)
      Serial.print(F("GET"));
    else if (request->method() == HTTP_POST)
      Serial.print(F("POST"));
    else if (request->method() == HTTP_DELETE)
      Serial.print(F("DELETE"));
    else if (request->method() == HTTP_PUT)
      Serial.print(F("PUT"));
    else if (request->method() == HTTP_PATCH)
      Serial.print(F("PATCH"));
    else if (request->method() == HTTP_HEAD)
      Serial.print(F("HEAD"));
    else if (request->method() == HTTP_OPTIONS)
      Serial.print(F("OPTIONS"));
    else
      Serial.print(F("UNKNOWN"));
    Serial.println(" http://" + request->host() + request->url());
    if (request->contentLength()) {
      Serial.println("_CONTENT_TYPE: " + request->contentType());
      Serial.println("_CONTENT_LENGTH: " + request->contentLength());
    }
    int headers = request->headers();
    int i;
    for (i = 0; i < headers; i++) {
      const AsyncWebHeader* h = request->getHeader(i);
      Serial.println("_HEADER[" + h->name() + "]: " + h->value());
    }
    int params = request->params();
    for (i = 0; i < params; i++) {
      const AsyncWebParameter* p = request->getParam(i);
      if (p->isFile()) {
        Serial.println("_FILE[" + p->name() + "]: " + p->value() + ", size: " + p->size());
      } else if (p->isPost()) {
        Serial.println("_POST[" + p->name() + "]: " + p->value());
      } else {
        Serial.println("_GET[" + p->name() + "]: " + p->value());
      }
    }
    request->send(404);
  }); // onNotFound

  server.begin();
  logTo::logToAll("HTTP server started @ " + WiFi.localIP().toString());
}
#endif