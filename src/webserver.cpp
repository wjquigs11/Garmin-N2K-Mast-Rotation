
#include "windparse.h"
#include "compass.h"
#include "BoatData.h"

// object-oriented classes
#include "BNO085Compass.h"
#include "logto.h"

Preferences preferences;     

// calibration; saved to preferences
int portRange=50, stbdRange=50; // NB BOTH are positive
#ifdef HONEY
extern Adafruit_ADS1015 ads;
extern int adsInit;
extern int PotValue;
#endif
int MagLo, MagHi; // ends of range corresponding to PotLo/PotHi and portRange/stbdRange
// however, they're just used as a calibration sanity check since they will change all the time when the boat is moving
#if 0 //CLEAN UP! it's in windparse.h
extern int mastOrientation; // mast compass position relative to boat compass position
extern int sensOrientation; // Honeywell orientation relative to centerline
extern int boatOrientation; // boat compass position relative to centerline
extern int mastAngle[];
extern float mastRotate, rotateout;
extern uint8_t compassAddress[];
extern float mastCompassDeg;
extern int boatCalStatus;
#endif
#ifdef PICAN
extern bool bmeFound;
extern Adafruit_BME280 bme;
#endif

// Create AsyncWebServer object on port 80
#define HTTP_PORT 80
AsyncWebServer server(HTTP_PORT);
AsyncEventSource events("/events");
//AsyncWebSocket ws("/ws");

// Json Variable to Hold Sensor Readings
JSONVar readings;

// Timer variables
unsigned long lastTime = 0;
int WebTimerDelay = 2000;

extern bool displayOnToggle, honeywellOnToggle, demoModeToggle;
//extern Adafruit_SSD1306 display;
const char* PARAM_INPUT_1 = "output";
const char* PARAM_INPUT_2 = "state";

#ifdef DEMO
void demoInit();
#endif
#ifdef BNO_GRV
int compassDifference(int angle1, int angle2);
#endif
// Get Sensor Readings and return JSON object
String getSensorReadings() {
  readings["rotateout"] = String(rotateout,0);
  readings["mastHeading"] = "";
  readings["mastDelta"] = "";
#ifdef HONEY
  if (honeywellOnToggle) {
    readings["mastRotate"] = mastAngle[0];
    readings["PotValue"] = PotValue;
  }
#endif
#ifdef BNO08X
  if (compass.OnToggle) {
    #ifdef BNO_GRV
    if (mastCompassDeg >= 0) { // set to -1 if mast compass times out
      readings["mastHeading"] = String(mastCompassDeg+mastOrientation,1);
      readings["mastDelta"] = mastAngle[1];
      readings["boatIMU"] = String(compass.boatIMU,1);
    }
    #endif
    readings["boatHeading"] = String(compass.boatHeading,0);
    readings["boatTrue"] = String(pBD->trueHeading,0);
    //readings["boatCalStatus"] = String(boatCalStatus);
    if (!honeywellOnToggle) // honeywell takes precedence if both are enabled
      readings["rotateout"] = String(rotateout,0);
  }
#endif
  readings["windSpeed"] = String(WindSensor::windSpeedKnots,2);
  readings["windAngle"] = String(WindSensor::windAngleDegrees,0);
#ifdef PICAN // doing this with URLs for now because I want the client side to control refresh rate
  if (bmeFound) {
    readings["temp"] = String(1.8 * bme.readTemperature() + 32);
    readings["pressure"] = String(bme.readPressure() / 100.0);
    readings["humidity"] = String(bme.readHumidity());
  }
#endif
  // note that there are also dedicated http endpoints for wind data for the wind.html charts
  // so this is a bit redundant
  // here, we're pushing updates via readings[]
  // but the dedicated endpoints are good for pulling updates on a different schedule
  // the problem with dedicated endpoints is that it crashes the ESP32 in asynctcp(), so going back to SSE
  readings["TWA"] = String(BoatData.TWA,2);
  readings["TWS"] = String(BoatData.TWS*MTOKTS,2);
  readings["VMG"] = String(BoatData.VMG*MTOKTS,2);
  String jsonString = JSON.stringify(readings);
  //log::toAll(jsonString);
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
  log::toAll("settings processor var: " + var);
  if (var == "BUTTONPLACEHOLDER") {
    settingsString = "<h4>Display</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"display\" ";
    if (displayOnToggle) settingsString += "checked";
    settingsString += "><span class=\"slider\"></span></label>";
    settingsString += "<h4>Honeywell</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"honeywell\" ";
    if (honeywellOnToggle) settingsString += "checked"; 
    settingsString += "><span class=\"slider\"></span></label>";
    settingsString += "<h4>Mast Compass</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"compass\" ";
#ifdef BNO_GRV
    if (compass.OnToggle) settingsString += "checked"; 
#endif
    settingsString += "><span class=\"slider\"></span></label>";
    return settingsString;
  }
  if (var == "webtimerdelay") return (settingsString = String(WebTimerDelay));
  if (var == "orientation") return (settingsString = String(mastOrientation));
  if (var == "sensorient") return (settingsString = String(sensOrientation));
  if (var == "boatorient") return (settingsString = String(boatOrientation));
  if (var == "RTKorient") return (settingsString = String(rtkOrientation));
#ifdef BNO08X
  if (var == "frequency") return (settingsString = String(compass.frequency));
#endif
  if (var == "controlMAC") return (settingsString = String(WiFi.macAddress()));
  if (var == "variation") return (settingsString = String(BoatData.Variation));
  return (settingsString = String("settings processor: placeholder not found " + var));
}

String demo_processor(const String& var) {
  log::toAll("demo processor var: " + var);
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
  log::toAll("toggleCheckbox id: " + String(id));
  if (strcmp(id, "display") == 0) {
    displayOnToggle = !displayOnToggle;
    log::toAll("setting display to " + String(displayOnToggle));
  }
  if (strcmp(id, "honeywell") == 0) {
    log::toAll("setting honeywell to " + String(honeywellOnToggle));
    honeywellOnToggle = !honeywellOnToggle;
  }
#ifdef BNO_GRV
  if (strcmp(id, "compass") == 0) {
    log::toAll("setting compass to " + String(compass.OnToggle));
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

void readPrefs() {
  preferences.begin("ESPwind", false);
  displayOnToggle = (preferences.getString("displayOnTog", "true") == "true") ? true : false;
  log::toAll("display = " + String(displayOnToggle));
#ifdef BNO08X
  compass.OnToggle = (preferences.getString("compass.OnTog", "false") == "true") ? true : false;
  log::toAll("compass = " + String(compass.OnToggle));
  //readings["compass"] = (compass.OnToggle ? 1 : 0);
  compass.frequency = preferences.getInt("compassFreq", 50);
  log::toAll("compassFrequency = " + String(compass.frequency));
  compass.reportType = preferences.getInt("rtype", 0);
  log::toAll("reportType = " + String(compass.reportType));
#endif
  honeywellOnToggle = (preferences.getString("honeywellOnTog", "false") == "true") ? true : false;
  log::toAll("honeywell = " + String(honeywellOnToggle));
  //readings["honeywell"] = (honeywellOnToggle ? 1 : 0);
  //demoModeToggle = (preferences.getString("demoModeTog", "false") == "true") ? true : false;
  log::toAll("demo = " + String(demoModeToggle));
  WebTimerDelay = preferences.getInt("WebTimerDelay", 500);
  //mastOrientation = preferences.getInt("mastOrientation", 0);
  sensOrientation = preferences.getInt("sensOrientation", 0);
  boatOrientation = preferences.getInt("boatOrientation", 0);
  BoatData.Variation = preferences.getFloat("variation", VARIATION);
#ifdef GPX
preferences.putString("GPXlog", "gpxfile");
preferences.putInt("GPXlogFileIdx", 0);
  strcpy(GPXlog, preferences.getString("GPXlog", "gpxfile").c_str());
  log::toAll("gpxFilePrefix = " + String(GPXlog));
  // increment here? maybe!
  logFileIdx = preferences.getInt("GPXlogFileIdx", 0)+1;
  log::toAll("GPXlogFileIdx = " + String(logFileIdx));
#endif
#ifdef WINDLOG
preferences.putString("windLog", windLog);
// comment out after first run
preferences.putInt("windLogFileIdx", 0);
strcpy(windLog, preferences.getString("windLog", windLog).c_str());
log::toAll("wind log file prefix = " + String(windLog));
// increment here? maybe
windLogFileIdx = preferences.getInt("windLogFileIdx", 0);//+1;
log::toAll("windLogFileIdx = " + String(windLogFileIdx));
// TBD: consider storing bool windLogging instead of turning it on each time you reboot
#endif
}

#ifdef PICAN
String readBME280Temperature() {
  // Read temperature as Celsius (the default)
  float t = bme.readTemperature();
  // Convert temperature to Fahrenheit
  t = 1.8 * t + 32;
  if (isnan(t)) {    
    Serial.println("Failed to read from BME280 sensor!");
    return "";
  }
  else {
    //Serial.printf("temp %0.0f\n", t);
    return String(t,0);
  }
}

String readBME280Humidity() {
  float h = bme.readHumidity();
  if (isnan(h)) {
    Serial.println("Failed to read from BME280 sensor!");
    return "";
  }
  else {
    //Serial.printf("humidity %0.0f\n", h);
    return String(h,0);
  }
}

String readBME280Pressure() {
  float p = bme.readPressure() / 100.0F;
  if (isnan(p)) {
    Serial.println("Failed to read from BME280 sensor!");
    return "";
  }
  else {
    //Serial.printf("pressure %0.0f\n", p);
    return String(p,0);
  }
}
#endif

void startWebServer() {
  log::toAll("starting web server");

  // Set up CORS headers
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Content-Type");

  if (!MDNS.begin(host.c_str()) ) {
    log::toAll("Error starting MDNS responder.");
  } else
      log::toAll("MDNS started " + host);

  // Add service to MDNS-SD
  if (!MDNS.addService("http", "tcp", HTTP_PORT)) {
    log::toAll("MDNS add service failed");
    sprintf(prbuf, "Free: heap %u, block: %u, min: %u, pSRAM %u", ESP.getFreeHeap(), ESP.getMaxAllocHeap(), ESP.getMinFreeHeap(), ESP.getFreePsram());
    log::toAll(prbuf);
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
    log::toAll("index.html");
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.on("/heap", HTTP_GET, [](AsyncWebServerRequest * request) {
    log::toAll("heap.html");
    request->send(200, "text/plain", String(ESP.getFreeHeap()));
  });
  
  server.on("/demo", HTTP_GET, [](AsyncWebServerRequest *request) {
    log::toAll("demo.html");
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
    //log::toAll("readings");
    String json;
    //json.reserve(512);
    json = getSensorReadings();
    //log::toAll("sending readings " + String(json.length()));
    request->send(200, "application/json", json);
    //log::toAll("readings sent");
    json = String();
  });

#ifdef PICAN
  if (bmeFound) {
    server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request) {
      //Serial.println("/temperature");
      request->send(200, "text/plain", readBME280Temperature().c_str());
    });
    server.on("/humidity", HTTP_GET, [](AsyncWebServerRequest *request) {
      //Serial.println("/humidity");
      request->send(200, "text/plain", readBME280Humidity().c_str());
    });
    server.on("/pressure", HTTP_GET, [](AsyncWebServerRequest *request) {
      //Serial.println("/pressure");
      request->send(200, "text/plain", readBME280Pressure().c_str());
    });
  }
  
  // Dedicated endpoints for Apparent Wind Speed and Apparent Wind Angle
  server.on("/aws", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", String(WindSensor::windSpeedKnots,2).c_str());
  });
  
  server.on("/awa", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", String(rotateout,2).c_str());
  });
  
  // Dedicated endpoints for True Wind Speed and True Wind Angle
  server.on("/tws", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", String(BoatData.TWS*MTOKTS,2).c_str());
  });
  
  server.on("/twa", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", String(BoatData.TWA,2).c_str());
  });
  
  // Dedicated endpoint for Speed Through Water
  server.on("/stw", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", String(BoatData.STW*MTOKTS,2).c_str());
  });
  
  // Dedicated endpoint for VMG to wind
  server.on("/vmg", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", String(BoatData.VMG*MTOKTS,2).c_str());
  });
  
  server.on("/weather", HTTP_GET, [](AsyncWebServerRequest *request) {
    log::toAll("weather.html");
    request->send(SPIFFS, "/weather.html", "text/html");
  });  
  
  server.on("/wind", HTTP_GET, [](AsyncWebServerRequest *request) {
    log::toAll("wind.html");
    request->send(SPIFFS, "/wind.html", "text/html");
  });
#endif

  server.on("/host", HTTP_GET, [](AsyncWebServerRequest *request) {
    String buf = "host: " + host + ", webtimerdelay: " + String(WebTimerDelay);
    log::toAll(buf);
    request->send(200, "text/plain", buf.c_str());
    buf = String();
  });

  server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *request) {
    log::toAll("settings.html");
    request->send(SPIFFS, "/settings.html", "text/html", false, settings_processor);
  });

  server.on("/compass", HTTP_GET, [](AsyncWebServerRequest *request) {
    log::toAll("compass.html");
    request->send(SPIFFS, "/compass.html", "text/html");
  });

  static int oldMastOrientation;
  server.on("/mastcompass", HTTP_GET, [](AsyncWebServerRequest *request) {
    log::toAll("mastcompass.html");
    String inputMessage1;
    String inputMessage2;
    oldMastOrientation = mastOrientation;
    mastOrientation = 0;
#ifdef BNO_GRV
    // GET input1 value on <ESP_IP>/params?output=<inputMessage1>&state=<inputMessage2>
    if (request->hasParam("confirm")) {
      inputMessage1 = request->getParam("confirm")->value();
      log::toAll("/mastcompass: " + inputMessage1);
      mastOrientation = compassDifference(compass.boatIMU, mastCompassDeg+mastOrientation);
    }
    if (request->hasParam("cancel")) {
      inputMessage1 = request->getParam("cancel")->value();
      log::toAll("/mastcompass: " + inputMessage1);
      mastOrientation = oldMastOrientation;
    }
#endif
    request->send(SPIFFS, "/mastcompass.html", "text/html");
  });

  // Send a GET request to <ESP_IP>/params?output=<inputMessage1>&state=<inputMessage2>
  server.on("/params", HTTP_GET, [] (AsyncWebServerRequest *request) {
    //log::toAll("params.html");
    String inputMessage1;
    String inputMessage2;
    // GET input1 value on <ESP_IP>/params?output=<inputMessage1>&state=<inputMessage2>
    if (request->hasParam(PARAM_INPUT_1) && request->hasParam(PARAM_INPUT_2)) {
      inputMessage1 = request->getParam(PARAM_INPUT_1)->value();
      inputMessage2 = request->getParam(PARAM_INPUT_2)->value();
      //digitalWrite(inputMessage1.toInt(), inputMessage2.toInt());
      //("/params got %s %s", inputMessage1, inputMessage2);
      log::toAll("/params: " + inputMessage1 + " " + inputMessage2);
      if (inputMessage1 == "display") {
        if (inputMessage2 == "off") {
          log::toAll("display off");
          displayOnToggle = false;
#ifdef DISPLAYON
          display->clearDisplay();
#endif
        } else {
          log::toAll("display on");
          displayOnToggle = true;
        }
        preferences.putString("displayOnTog", displayOnToggle ? "true" : "false");
      }
#ifdef BNO_GRV
      if (inputMessage1 == "compass") {
        if (inputMessage2 == "off") {
          log::toAll("compass off");
          compass.OnToggle = false;
          readings["compass"] = 0;
        } else {
          log::toAll("compass on");
          compass.OnToggle = true;
          readings["compass"] = 1;
        }
        preferences.putString("compass.OnTog", compass.OnToggle ? "true" : "false");
        //sendMastControl();
      }
#endif
      if (inputMessage1 == "honeywell") {
        if (inputMessage2 == "off") {
          log::toAll("honeywell off");
          honeywellOnToggle = false;
          readings["honeywell"] = 0;
        } else {
          log::toAll("honeywell on");
          honeywellOnToggle = true;
          readings["honeywell"] = 1;
        }
        preferences.putString("honeywellOnTog", honeywellOnToggle ? "true" : "false");
       }
#ifdef DEMO
      if (inputMessage1 == "demo") {
        if (inputMessage2 == "off") {
          log::toAll("demo off");
          demoModeToggle = false;
        } else {
          log::toAll("demo on");
          demoModeToggle = true;
          demoInit();
        }
        preferences.putString("demoModeTog", demoModeToggle ? "true" : "false");
      }
#endif
    }
    else {
      inputMessage1 = "No message sent";
      inputMessage2 = "No message sent";
    }
    request->send(200, "text/plain", "OK");
    inputMessage1 = String();
    inputMessage2 = String();
  });

#if 0
  server.on("/calibrate", HTTP_GET, [](AsyncWebServerRequest *request) {
    log::toAll("calibrate.html");
    request->send(SPIFFS, "/calibrate.html", "text/html", false, cal_processor);
  });

  // TBD: modify pages to reflect honeywell and compass toggles
  // POST on calibrate means we've gotten rotation range parameters
  server.on("/calibrate", HTTP_POST, [](AsyncWebServerRequest *request) {
    int params = request->params();
    for(int i=0;i<params;i++) {
      const AsyncWebParameter* p = request->getParam(i);
      if(p->isPost()) {
        // HTTP POST ssid value
        if (p->name() == "portRange") {
          portRange = atoi(p->value().c_str());
          preferences.putInt("portRange", portRange);
        }
        if (p->name() == "stbdRange") {
          stbdRange = atoi(p->value().c_str());
          preferences.putInt("stbdRange", stbdRange);
        }
      } // isPost
    } // for params
    request->send(SPIFFS, "/calibrate2.html", "text/html", false, cal_processor); // go to 2nd page
  });

  // POST on calibrate2 means mast is all the way to port
  server.on("/calibrate2", HTTP_POST, [](AsyncWebServerRequest *request) {
    //Serial.println("calibrate post");
    /*
    int params = request->params();
    Serial.printf("/calibrate POST got %d params", params);
    for(int i=0;i<params;i++) {
      AsyncWebParameter* p = request->getParam(i);
      if(p->isPost()) {
        Serial.printf("param: %s %s", p->name(), p->value());
      }
    }*/
#ifdef HONEY
    // RN post isn't including PotValue actual value so I'll just read it and hope they don't move the mast
    if (adsInit)
      preferences.putInt("Honeywell.left", ads.readADC_SingleEnded(0));
    if (mastAngle[1])
      preferences.putInt("Mast.left", mastAngle[1]);
#endif
    request->send(SPIFFS, "/calibrate3.html", "text/html");
  });

  // POST on calibrate3 means mast is all the way to starboard
  server.on("/calibrate3", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (adsInit)
      preferences.putInt("Honeywell.right", ads.readADC_SingleEnded(0));    
    if (mastAngle[1])
      preferences.putInt("Mast.right", mastAngle[1]);
    request->send(SPIFFS, "/calibrate4.html", "text/html");
  });

  // POST on calibrate4 means mast should be centered and user has pressed OK; save values
  // (cancel will redirect to /index.html)
  server.on("/calibrate4", HTTP_POST, [](AsyncWebServerRequest *request) {
    //preferences.putInt("portRange", portRange);
    //preferences.putInt("stbdRange", stbdRange);
    // what now? If you put a dial in calibrate4, maybe just refresh, but delete the buttons
    // or put the dial on index.html with a Calibrate button
    // check the difference between mast compass and boat compass; set orientation
    request->send(SPIFFS, "/index.html", "text/html");
  });
#endif

  // POST on params
  server.on("/params", HTTP_POST, [](AsyncWebServerRequest *request) {
    int params = request->params();
    for(int i=0;i<params;i++) {
      const AsyncWebParameter* p = request->getParam(i);
      if(p->isPost()) {
        // HTTP POST ssid value
        log::toAll("params POST " + p->name() + " " + p->value());
        if (p->name() == "webtimerdelay") {
          WebTimerDelay = atoi(p->value().c_str());
          preferences.putInt("WebTimerDelay", WebTimerDelay);
        }
        if (p->name() == "sensorient") {
          sensOrientation = atoi(p->value().c_str());
          preferences.putInt("sensOrientation", sensOrientation);
        }
        if (p->name() == "orientation") {
          mastOrientation = atoi(p->value().c_str());
          preferences.putInt("mastOrientation", mastOrientation);
        }
#ifdef BNO08X
        if (p->name() == "frequency") {
          compass.frequency = atoi(p->value().c_str());
          preferences.putInt("compassFreq", compass.frequency);
        }
#endif
        if (p->name() == "variation") {
          BoatData.Variation = atof(p->value().c_str());
          preferences.putFloat("variation", BoatData.Variation);
        }
        //sendMastControl();  // notify mast compass via ESPNOW
        if (p->name() == "boatorient") {
          boatOrientation = atoi(p->value().c_str());
          preferences.putInt("boatOrientation", boatOrientation);
        }
        if (p->name() == "RTKorient") {
          rtkOrientation = atoi(p->value().c_str());
          preferences.putInt("rtkOrientation", rtkOrientation);
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
  log::toAll("HTTP server started @ " + WiFi.localIP().toString());
}

HTTPClient httpC;
char mastCompassURL[] = "http://mastcomp.local/readings";
JSONVar mastCompRead;
String jsonString;
bool isConnected = false;

float getMastHeading() {
    // ping mast compass needs work for use case where mast compass isn't connected to external AP
    if(WiFi.status() == WL_CONNECTED) {  
      if (isConnected) {
        Serial.printf("ts0=%ld\n",millis());
        //httpGETRequest
        int httpResponseCode = httpC.GET();
        Serial.printf("ts1=%ld\n",millis());
        if (httpResponseCode>0) {
          //Serial.print("HTTP Response code: ");
          //Serial.println(httpResponseCode);
          jsonString = httpC.getString();
          //Serial.println(jsonString);
          JSONVar myObject = JSON.parse(jsonString);
          if (JSON.typeof(myObject) == "undefined") {
            Serial.println("Parsing input failed!");
            return -1.0;
          }
          //float bearing = JSON.stringify(myObject["bearing"]).toFloat();
          String bS = JSON.stringify(myObject["bearing"]);
          bS.replace("\"", ""); // Remove quotes
          float bearing = bS.toFloat();
          int variation = myObject["variation"];
          int orientation = myObject["orientation"];
          int frequency = myObject["frequency"];
          int calstatus = myObject["calstatus"];
          //Serial.printf("%s %0.2f %d %d %d %d\n", bS.c_str(), bearing, variation, orientation, frequency, calstatus);
          return bearing;
        }
      } else {
        if (httpC.begin(mastCompassURL)) {
          isConnected = true;
          // all good but wait until next time
        } else Serial.println("Unable to connect");
      }
     } else Serial.println("WiFi Disconnected");
    return -2.0;
}
