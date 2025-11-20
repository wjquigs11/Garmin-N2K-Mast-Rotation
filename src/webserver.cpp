#if defined(WIFI)
#include "include.h"

// Include AsyncWebServer headers directly for compilation
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
AsyncWebServer server(HTTP_PORT);
AsyncEventSource events("/events");
//AsyncWebSocket ws("/ws");

bool serverStarted;
JsonDocument readings;

String host = "ESPtemplate";

#ifdef TEMPLATE
String getSensorReadings() {
  readings["sensor"] = "0";
  String jsonString;
  serializeJson(readings,jsonString);
  return jsonString;
}

String processor(const String& var) {
  Serial.println(var);
  if(var == "TIMERDELAY") {
    return String(timerDelay);
  }
  return String();
}
#endif

void startWebServer() {
  log::toAll("starting web server");

  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    log::toAll("index.html");
    request->send(SPIFFS, "/index.html", "text/html", false, [](const String& var) { return processor(var); });
  });

  // start serving from SPIFFS
  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");
  server.serveStatic("/", SPIFFS, "/");

  // Request latest sensor readings
  server.on("/readings", HTTP_GET, [](AsyncWebServerRequest *request) {
    AsyncWebServerResponse *response = request->beginResponse(200, "application/json", getSensorReadings());
    // Add CORS headers
    response->addHeader("Access-Control-Allow-Origin", "*");
    response->addHeader("Access-Control-Allow-Methods", "GET");
    response->addHeader("Access-Control-Allow-Headers", "Content-Type");
    request->send(response);
  });

  server.on("/host", HTTP_GET, [](AsyncWebServerRequest *request) {
    String buf = "hostname: " + host;
    buf += ", ESP local MAC addr: " + String(WiFi.macAddress());
    log::toAll(buf);
    request->send(200, "text/plain", buf.c_str());
    buf = String();
  });

#ifdef TEMPLATE
  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request) {
    log::toAll("config:");
    String response = "none";
    if (request->hasParam("hostname")) {
      Serial.printf("hostname %s", request->getParam("hostname")->value().c_str());
      host = request->getParam("hostname")->value();
      response = "change hostname to " + host;
      log::toAll(response);
      preferences.putString("hostname",host);
      log::toAll("preferences " + preferences.getString("hostname", "unknown"));
    } else if (request->hasParam("webtimer")) {
      timerDelay = atoi(request->getParam("webtimer")->value().c_str());
      if (timerDelay < 0) timerDelay = DEFDELAY;
      if (timerDelay > 10000) timerDelay = 10000;
      response = "change web timer to " + String(timerDelay);
      log::toAll(response);
      preferences.putInt("timerdelay",timerDelay);
    } else if (request->hasParam("value")) {
      // do something with "value" parameter
      response = "value successful";
      log::toAll(response);
    }
    request->send(200, "text/plain", response.c_str());
    response = String();
  });
#endif

#ifdef NTP
  // Always register the /clienttime endpoint to avoid 404 errors
  server.on("/clienttime", HTTP_POST, [](AsyncWebServerRequest *request){},
    NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      // Check if NTP sync was successful
      if (!isNtpSyncSuccessful()) {
        // Only use client time if NTP sync failed
        log::toAll("Using client time as NTP sync failed");
        JsonDocument doc;
        ::deserializeJson(doc, data);
        String clientTime = doc["datetime"];
        Serial.print("Received client time: ");
        Serial.println(clientTime);
        
        int year, month, day, hour, minute, second;
        sscanf(clientTime.c_str(), "%4d-%2d-%2dT%2d:%2d:%2d", &year, &month, &day, &hour, &minute, &second);
        struct tm timeinfo;
        timeinfo.tm_year = year - 1900;   // tm_year: years since 1900
        timeinfo.tm_mon  = month - 1;     // tm_mon: months since January [0,11]
        timeinfo.tm_mday = day;           // tm_mday: day of the month [1,31]
        timeinfo.tm_hour = hour;
        timeinfo.tm_min  = minute;
        timeinfo.tm_sec  = second;
        time_t t = mktime(&timeinfo);  // Convert to time_t (seconds since epoch)
        struct timeval now = { .tv_sec = t, .tv_usec = 0 };
        settimeofday(&now, NULL);      // Set the ESP32 system clock
        request->send(200, "text/plain", "Time received and set");
      } else {
        // If NTP sync was successful, acknowledge but don't use the client time
        log::toAll("Received client time but using NTP time instead");
        request->send(200, "text/plain", "Time received but using NTP time");
      }
      
      // Clean up
      String clientTime = String();
    }
  );
#endif

  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 1000);
  });
  server.addHandler(&events);
  //server.addHandler(&ws);

  // Handle OPTIONS preflight requests for CORS
  server.onNotFound([](AsyncWebServerRequest *request) {
    if (request->method() == HTTP_OPTIONS) {
      AsyncWebServerResponse *response = request->beginResponse(204);
      response->addHeader("Access-Control-Allow-Origin", "*");
      response->addHeader("Access-Control-Allow-Methods", "GET, OPTIONS");
      response->addHeader("Access-Control-Allow-Headers", "Content-Type");
      response->addHeader("Access-Control-Max-Age", "3600");
      request->send(response);
      return;
    }
    // Handle not found for other requests
    request->send(404);
  });
}
#endif