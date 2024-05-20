
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
#include <Arduino_JSON.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <Adafruit_ADS1X15.h>
#include <N2kMessages.h>
#include "windparse.h"

extern String host;

Preferences preferences;     

// calibration; saved to preferences
int portRange=50, stbdRange=50; // NB BOTH are positive
extern Adafruit_ADS1015 ads;
extern int adsInit;
extern int PotValue;
int MagLo, MagHi; // ends of range corresponding to PotLo/PotHi and portRange/stbdRange
// however, they're just used as a calibration sanity check since they will change all the time when the boat is moving
extern int magOrientation; // we *should* start getting valid magOrientation as soon as BLE is connected
extern int mastRotate, rotateout;
extern int mastAngle[];

// Create AsyncWebServer object on port 80
//AsyncWebServer server(80);
// Create an Event Source on /events
extern AsyncEventSource events;
extern AsyncWebServer server;

// Json Variable to Hold Sensor Readings
JSONVar readings;

// Timer variables
unsigned long lastTime = 0;
int WebTimerDelay = 500;

extern char buf[];
extern bool displayOnToggle;
const char* PARAM_INPUT_1 = "output";
const char* PARAM_INPUT_2 = "state";

// Get Sensor Readings and return JSON object
String getSensorReadings() {
  // speed/angle/rotateout are assigned in windparse.cpp
  // magnetic heading/mast heading are assigned in magheading.cpp
  // switched to cal_processor but that's not dynamic so switching back
  if (adsInit)
      PotValue = ads.readADC_SingleEnded(0);
  readings["PotValue"] = String(PotValue);
  String jsonString = JSON.stringify(readings);
  //Serial.println(readings);
  return jsonString;
}

// Replaces HTML %placeholder% with stored values, called when page renders
String settings_processor(const String& var) {
  Serial.printf("settings processor var: %s\n", var.c_str());
  if(var == "DISPLAYSTATE") {
    if (displayOnToggle)
      return String("on");
    else
      return String("off");
  }
  return String();
}

// Replaces HTML %placeholder% with stored values
String cal_processor(const String& var) {
  //Serial.printf("cal processor var: %s\n", var.c_str());
  if (var == "portRange") {
    return String(preferences.getInt("portRange"));
  }
  if (var == "stbdRange") {
    return String(preferences.getInt("stbdRange"));
  }
  /* works BUT does not update value continuously, maybe change to javascript/readings?
  if (var == "PotValue") {
    if (adsInit) {
      return String(ads.readADC_SingleEnded(0));
    } else return String("55");
  }*/
  return String("cal processor: placeholder not found " + var);
}

void startWebServer() {

  preferences.begin("calibration", false);

  // start serving from SPIFFS
  server.serveStatic("/", SPIFFS, "/");
  
  server.on("/demo", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/demo.html", "text/html");
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
    String json = getSensorReadings();
    request->send(200, "application/json", json);
    json = String();
  });

  server.on("/host", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.print("hostname: ");
    Serial.println(host.c_str());
    //sprintf(buf, "host: %s, variation: %d, orientation: %d, timerdelay: %d", host.c_str(), variation, orientation, timerDelay);
    sprintf(buf, "host %s", host.c_str());
    request->send_P(200, "text/plain", buf);
  });

  server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/settings.html", "text/html", false, settings_processor);
    //request->send(SPIFFS, "/settings.html", "text/html");
  });

  // Send a GET request to <ESP_IP>/update?output=<inputMessage1>&state=<inputMessage2>
  server.on("/update", HTTP_GET, [] (AsyncWebServerRequest *request) {
    //Serial.println("update");
    String inputMessage1;
    String inputMessage2;
    // GET input1 value on <ESP_IP>/update?output=<inputMessage1>&state=<inputMessage2>
    if (request->hasParam(PARAM_INPUT_1) && request->hasParam(PARAM_INPUT_2)) {
      inputMessage1 = request->getParam(PARAM_INPUT_1)->value();
      inputMessage2 = request->getParam(PARAM_INPUT_2)->value();
      //digitalWrite(inputMessage1.toInt(), inputMessage2.toInt());
      //Serial.printf("/update got %s %s\n", inputMessage1, inputMessage2);
      if (inputMessage1 == "display") {
        if (inputMessage2 == "off")
          displayOnToggle = false;
        else
          displayOnToggle = true;
      }
    }
    else {
      inputMessage1 = "No message sent";
      inputMessage2 = "No message sent";
    }
    request->send(200, "text/plain", "OK");
  });

  server.on("/calibrate", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/calibrate.html", "text/html", false, cal_processor);
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
    Serial.printf("/calibrate POST got %d params\n", params);
    for(int i=0;i<params;i++) {
      AsyncWebParameter* p = request->getParam(i);
      if(p->isPost()) {
        Serial.printf("param: %s %s\n", p->name(), p->value());
      }
    }*/
    // RN post isn't including PotValue actual value so I'll just read it and hope they don't move the mast
    if (adsInit)
      preferences.putInt("Honeywell.left", ads.readADC_SingleEnded(0));
    if (mastAngle[1])
      preferences.putInt("Mast.left", mastAngle[1]);
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

  // POST on calibrate4 means user has pressed OK; save values
  // (cancel will redirect to /index.html)
  server.on("/calibrate4", HTTP_POST, [](AsyncWebServerRequest *request) {
    //preferences.putInt("portRange", portRange);
    //preferences.putInt("stbdRange", stbdRange);
    // what now? If you put a dial in calibrate4, maybe just refresh, but delete the buttons
    // or put the dial on index.html with a Calibrate button
    request->send(SPIFFS, "/index.html", "text/html");
  });
}

void initWebSocket() {
  server.addHandler(&events);
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
