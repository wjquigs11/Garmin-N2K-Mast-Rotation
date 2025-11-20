
#include "include.h"
#include "windparse.h"

File consLog;
Preferences preferences;

bool wifiEnabled = true;
bool wifiConnected = false;
int timerDelay = 1000; // this sets loop time after housekeeping tasks are done
int loopDelay = 10;
time_t lastUpdate, updateTime;
unsigned long lastTime = 0;
struct tm *ptm;
char prbuf[PRBUF]; // PRBUF needs to be defined in include.h

#ifdef REACT
using namespace reactesp;
ReactESP app;
#endif

#ifdef DEEPSLEEP
// Deep sleep variables
int awakeTimer = 300;  // stay awake for X seconds each time you wake up
// RTC memory variables (persist across deep sleep)
RTC_DATA_ATTR int bootCount = 0;

// Function to print the reason by which ESP32 has been awaken from sleep
void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0: log::toAll("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1: log::toAll("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER: log::toAll("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: log::toAll("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP: log::toAll("Wakeup caused by ULP program"); break;
    default: log::toAll("Wakeup was not caused by deep sleep: " + String((int)wakeup_reason)); break;
  }
}
#else
int bootCount = 0;
#endif
unsigned long startTime; // Time when the device started

void setup() {
  Serial.begin(115200); delay(300);
  startTime = millis();

  // Enable WiFi debugging
  //Serial.println("Enabling WiFi debugging...");
  //esp_log_level_set("wifi", ESP_LOG_VERBOSE);

  if (SPIFFS.begin()) {
    Serial.println("opened SPIFFS");
    size_t totalBytes = SPIFFS.totalBytes();
    size_t usedBytes = SPIFFS.usedBytes();
    Serial.print("SPIFFS Total space: ");
    Serial.print(totalBytes);
    Serial.println(" bytes");
    Serial.print("SPIFFS Used space: ");
    Serial.print(usedBytes);
    Serial.println(" bytes");
    Serial.print("SPIFFS Free space: ");
    Serial.print(totalBytes - usedBytes);
    Serial.println(" bytes");
    float usedPercentage = ((float)usedBytes / totalBytes) * 100;
    Serial.print("SPIFFS Usage: ");
    Serial.print(usedPercentage);
    Serial.println("%");
  } else {
    Serial.println("failed to open SPIFFS");
  }

  consLog = SPIFFS.open("/console.log", "a", true);
  if (!consLog) {
    log::toAll("failed to open console log");
  }
  if (consLog.println("ESP console log.")) {
    log::toAll("console log written");
  } else {
    log::toAll("console log write failed");
  }

  // Increment boot number and print it every reboot
  ++bootCount;
  log::toAll("Boot number: " + String(bootCount));

  // initialize N2K wind bus
  setupWind();

#ifdef DEEPSLEEP
  // Print the wakeup reason for ESP32
  print_wakeup_reason();

  // Configure the wake up source - set ESP32 to wake up every TIME_TO_SLEEP seconds
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  log::toAll("Setup ESP32 to sleep for " + String(TIME_TO_SLEEP) + " Seconds every " + String(awakeTimer) + " seconds");
#endif

  preferences.begin("ESPprefs", false);
  timerDelay = preferences.getInt("timerdelay", 1000);
  if (timerDelay<200) {
    timerDelay = 200;
    preferences.putInt("timerdelay", 1000);
  }
  log::toAll("timerDelay " + String(timerDelay) + " ms");
#ifdef REACT
  app.onRepeat(1, []() {
    loopWind();
  });

  //app.onRepeat(timerDelay, []() {
  //  log::toAll("rotateout: " + String(rotateout));
  //});
#endif
#ifdef WIFI
  wifiEnabled = preferences.getBool("wifi", true);
  host = preferences.getString("hostname", host);
  log::toAll("hostname: " + host);
  bool doubleReset = preferences.getBool("DRD", false);
  preferences.putBool("DRD", true);
  if (wifiEnabled) {
    if (doubleReset) {
      log::toAll("double reset detected");
      preferences.putBool("DRD", false);
      preferences.end();
      resetWifi();
    } else {
      // set up DRD if another reboot happens in 10 seconds
      preferences.putBool("DRD", true);
      wifiConnected = setupWifi();
    }
    // Start the web server regardless of WiFi connection status
    // In fallback mode, it will serve from AP mode
    startWebServer();
    serverStarted = true;
    
#ifdef ELEGANTOTA
    ElegantOTA.begin(&server);
#endif
#ifdef WEBSERIAL
    WebSerial.begin(&server);
    // Attach a callback function to handle incoming messages
    WebSerial.onMessage(WebSerialonMessage);
#endif
    if (wifiConnected) {
      log::toAll("HTTP server started @" + WiFi.localIP().toString());
    } else {
      log::toAll("HTTP server started in AP mode @" + WiFi.softAPIP().toString());
    }
    if (!MDNS.begin(host.c_str()))
      log::toAll(F("Error starting MDNS responder"));
    else {
      log::toAll("MDNS started " + host);
    }
    // Add service to MDNS-SD
    if (!MDNS.addService("http", "tcp", HTTP_PORT))
      log::toAll("MDNS add service failed");
    int n = MDNS.queryService("http", "tcp");
    if (n == 0) {
      log::toAll("No services found");
    } else {
      for (int i = 0; i < n; i++) {
        log::toAll("mdns service: " + MDNS.hostname(i));
      }
    }
  }
#endif // WIFI
  consLog.flush();
}

void loop() {
#ifdef REACT
  app.tick();
#endif
#ifdef ELEGANTOTA
  ElegantOTA.loop();
#endif
#ifdef WEBSERIAL
  WebSerial.loop();
#endif
  unsigned long now = millis();
  static unsigned long lastEventTime, lastTimeTime, startTime;
#ifdef WIFI
  dnsServer.processNextRequest(); // for captive portal
  // update web page
  static bool drdCleared = false;
  
  // Clear DRD flag after DRD_TIMEOUT seconds for double reset detection
  // unless reset occurs within the timeout period
  if (!drdCleared && (now > (DRD_TIMEOUT * 1000))) {
    preferences.putBool("DRD", false);
    drdCleared = true;
    log::toAll("DRD timeout - cleared double reset flag");
  }
#endif
// this all needs to move to app.onRepeat in setup()
    //if (now - lastEventTime > timerDelay || lastEventTime == 0) {
    //  lastEventTime = now;
#if defined(WIFI) && defined(NTP) 
    // Get current time from NTP-synchronized system clock if available
    if (isNtpSyncSuccessful()) {
      // Use the NTP-synchronized time
      lastUpdate = getEpochTime();
    } else if (updateTime > 0) {
      // Fallback: Use the browser's timestamp as base and add elapsed milliseconds
      lastUpdate = updateTime + (now - lastEventTime);
    } else {
      // If neither NTP nor browser time is available, use millis() as last resort
      // This will be incorrect since ESP32 millis() is not a Unix timestamp
      lastUpdate = now;
    }
    
    // kind of annoying that it resyncs right after startup sync but I guess I can live with it for now
    // Periodically resync NTP time (once per day)
    static unsigned long lastNTPSync = 0;
    if (isNtpSyncSuccessful() && (now - lastNTPSync > 86400000 || lastNTPSync == 0)) {
      resyncNTP();
      lastNTPSync = now;
    }
    
    // Send the timestamp in milliseconds since epoch (Unix timestamp)
    readings["lastUpdate"] = String(lastUpdate);
    events.send(getSensorReadings().c_str(),"new_readings" ,millis());
    // Use system time directly instead of lastUpdate
    //time_t now_time;
    //time(&now_time);
    //ptm = localtime(&now_time);
    //sprintf(prbuf,"[%02d/%02d %02d:%02d:%02d] ",ptm->tm_mon+1,ptm->tm_mday,ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
    //log::toAll(String(prbuf));
  #endif // WIFI
  consLog.flush();

  if (Serial.available() > 0) {
    String input = "";
    while (Serial.available() > 0) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        // Ignore newline and carriage return characters
        break;
      }
      input += c;
    }
    if (input.length() > 0) {
      Serial.print("Received: ");
      Serial.println(input);
      if (input == "value") {
        // do something with serial input
        Serial.println("Value command received");
      }
    }
  }
    
#ifdef DEEPSLEEP
    // Check if it's time to go to sleep
    if ((millis() - startTime) > (awakeTimer * 1000)) {
      log::toAll("Going to sleep in 5 seconds...");
      
      // Flush any pending data
      consLog.flush();
  #ifdef WEBSERIAL
      WebSerial.flush();
  #endif
      
      // Give time for final communications
      delay(5000);
      
      // Enter deep sleep
      log::toAll("Entering deep sleep for " + String(TIME_TO_SLEEP) + " seconds");
      esp_deep_sleep_start();
      // Code after this point will not be executed
    }
#endif // DEEPSLEEP
} // loop()
