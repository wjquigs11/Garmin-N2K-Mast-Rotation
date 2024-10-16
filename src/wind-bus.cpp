/* 
wind-bus.cpp
Dual-ESP mast rotation solution
ESP1 (CYD) is connected to 2 IMUs (one on mast, one in controller box), and a Hall-effect sensor
ESP1 is connected to an isolated wind N2K bus to keep uncorrected wind data from Garmin displays
ESP1 corrects incoming wind data for rotation and forwards in Actisense format on UART
TBD: ESP1 stores AWA, AWS, and STW on SD card for polars
ESP2 (SH-ESP32) is connected to ESP1 via UART, receives correct wind data (Actisense), and transmits on main N2K bus
*/
#ifdef WINDBUS
#include "compass.h"
#include "windparse.h"
#include "wind-bus.h"
#include "BoatData.h"
//#include <ElegantOTA.h>

// object-oriented classes
#include "logto.h"
#ifdef N2K
#include "n2k.h"
#endif

#define CAN_TX_PIN GPIO_NUM_22
#define CAN_RX_PIN GPIO_NUM_27

#ifdef SH_ESP32
//#define CAN_TX_PIN GPIO_NUM_32
//#define CAN_RX_PIN GPIO_NUM_34
// external transceiver because on-board one isn't working
#define CAN_RX_PIN GPIO_NUM_27
#define CAN_TX_PIN GPIO_NUM_25
//#define N2k_CAN_INT_PIN 22   
//#define MOSI 23
//#define MISO 19
//#define SS 5
//#define SCK 18
#define SDA_PIN 16
#define SCL_PIN 17
//#define TXD0 18
//#define RXD0 19
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
// RPI pin 24 = CP0/GPIO8 = IO5
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define SCREEN_WIDTH 128  
#define SCREEN_HEIGHT 64  
#endif // SH_ESP32

#ifdef CYD
#include <lvgl.h>
#include <TFT_eSPI.h>
void setupLVGL(); 
void LVGLdataWindDebug();
#endif

using namespace reactesp;
ReactESP app;
#define RECOVERY_RETRY_MS 1000  // How long to attempt CAN bus recovery
bool stackTrace = true;
//TwoWire *i2c;

tBoatData *pBD;
tBoatData BoatData;

extern Stream *read_stream;
extern Stream *forward_stream;

tNMEA2000 *n2kMain;
//bool n2kMainOpen = false;
//extern tN2kMsg correctN2kMsg;

/* timing/display
int num_n2k_messages = 0;
int num_wind_messages = 0;
int num_wind_fail = 0;
int num_wind_other = 0;
int num_wind_other_fail = 0;
int num_wind_other_ok = 0;
int num_mastcomp_messages = 0;
elapsedMillis time_since_last_can_rx = 0;
elapsedMillis time_since_last_wind_rx = 0;
unsigned long total_time_since_last_wind = 0.0;
unsigned long avg_time_since_last_wind = 0.0;
elapsedMillis time_since_last_mastcomp_rx = 0;
unsigned long total_time_since_last_mastcomp = 0.0;
unsigned long avg_time_since_last_mastcomp = 0.0;
*/
// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

// defs for wifi
void initWebSocket();
//void notifyClients(String);
extern AsyncWebServer server;
bool serverStarted=false;
extern char *hostname;
extern int WebTimerDelay;
extern AsyncEventSource events;
//extern JSONVar readings;
extern JsonDocument readings;
bool initWiFi();
void startAP();
void startWebServer();
bool sendMastControl();
String getSensorReadings();

String host = "ESPwind";

void httpInit(const char* serverName);
extern const char* serverName;
//int mastOrientation; // delta between mast compass and boat compass
//int sensOrientation; // delta between mast centered and Honeywell sensor reading at center
//int boatOrientation; // delta between boat compass and magnetic north
//float boatIMUdeg; // magnetic heading not corrected for variation
//float boatCompassTrue;
//float mastIMUdeg;
//float mastDelta;
//movingAvg mastCompDelta(10);
void mastHeading();
extern int compassFrequency;
int mastAngle;

bool imuReady=false;
bool mastIMUready=false;

#ifdef N2K
#include "n2k.h"
#endif

int headingErrCount;

// Time after which we should reboot if we haven't received any CAN messages
#define MAX_RX_WAIT_TIME_MS 30000

bool displayOnToggle=true, demoModeToggle=false;
//extern bool teleplot;

File consLog;

//void WebSerialonMessage(uint8_t *data, size_t len);
void i2cScan(TwoWire Wire);
//void demoIncr();
float readCompassDelta();

int compassDifference(int angle1, int angle2);

//bool setupWindBus();

void setup() {
  Serial.begin(115200); delay(500);
  logTo::logToAll("starting wind correction");

  esp_log_level_set("*", ESP_LOG_VERBOSE);  // Set global log level

  //adc1_config_width(ADC_WIDTH_BIT_12);
  //adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_MAX);

  Wire.begin();
  //Wire.setClock(100000);

#ifdef CYD
  //Serial.printf("display start\n");
  setupLVGL();
#endif
  // toggle the LED pin at rate of 1 Hz
  pinMode(LED_BUILTIN, OUTPUT);
  pBD=&BoatData;
#ifdef N2K
  app.onRepeatMicros(1e6 / 1, []() { n2k::ToggleLed(); });
  if (n2k::setupWindBus())
    logTo::logToAll("wind bus active");
#endif
  if (!SPIFFS.begin())
    logTo::logToAll("An error has occurred while mounting SPIFFS");
  else
    logTo::logToAll("SPIFFS mounted successfully");

  // start a console.log file in case we crash before Webserial starts
  if (SPIFFS.exists("/console.log")) {
    SPIFFS.remove("/console.log");
  }
  consLog = SPIFFS.open("/console.log", "w", true);
  if (!consLog) {
    logTo::logToAll("failed to open console log");
  }
  if (consLog.println("ESP compass console log.")) {
    logTo::logToAll("console log written");
  } else {
    logTo::logToAll("console log write failed");
  }
  if (initWiFi()) {
    startWebServer();
  } else {
    startAP();
  }
  serverStarted=true;

  logTo::logToAll("ESP local MAC addr: " + WiFi.macAddress());
  // TBD: decide if you want wifi on MAINBUS for WebSerial console
  // or just send everything over UART link, since you will (probably) not be reading CAN from MAINBUS
  //ElegantOTA.begin(&server);
  //WebSerial.begin(&server);
  //WebSerial.onMessage(WebSerialonMessage);

  // Update the time
  #define RETRIES 10
  int count=0;
  while(!timeClient.update() && count++ < RETRIES) {
      timeClient.forceUpdate();
  }
  logTo::logToAll(timeClient.getFormattedDate());
  // imuReady means we have a local boat compass in the controller

  logTo::logToAll("ESP flash size 0x" + String(ESP.getFlashChipSize(),HEX)); // 4194304

  consLog.flush();
#ifdef N2K
  // No need to parse the messages at every single loop iteration; 1 ms will do
  app.onRepeat(1, []() {
    n2k::PollCANStatus();
    n2k::n2kWind->ParseMessages();
  });
#endif
  // update web page
  app.onRepeat(WebTimerDelay, []() {
    static int counter;
    //logTo::logToAll("transmit sensor readings");
    // Send Events to the client with the Sensor Readings Every x seconds
    //events.send("ping",NULL,millis());
    events.send(getSensorReadings().c_str(),"new_readings" ,millis());
//    mastIMUdeg = getMastHeading();
    if (counter++ % 600 == 0) {
      // check that timeClient isn't hanging the whole system
      logTo::logToAll(timeClient.getFormattedDate());
    }
    consLog.flush();
  });

#if 0
  app.onRepeat(10, []() {
    //ElegantOTA.loop();  
    WebSerial.loop();
  });
#endif
// if wifi not connected, we're only going to attempt reconnect once every 5 minutes
// if we get in range, it's simpler to reboot than to constantly check
  app.onRepeat(300000, []() {
    if (WiFi.status() != WL_CONNECTED) {
      logTo::logToAll("WiFi not connected");
      initWiFi();
      if (WiFi.status() == WL_CONNECTED) {
        startWebServer();
        serverStarted=true;
      }
    }
    // check for connected clients
    int numClients = WiFi.softAPgetStationNum();
    logTo::logToAll("Number of connected clients: " + String(numClients));
    wifi_sta_list_t stationList;
    esp_wifi_ap_get_sta_list(&stationList);
    for (int i = 0; i < stationList.num; i++) {
      wifi_sta_info_t station = stationList.sta[i];
      logTo::logToAll("Client " + String(i+1) + " MAC: " + String(station.mac[0],HEX) + ":" + String(station.mac[1],HEX)+ ":" + String(station.mac[2],HEX)+ ":" + String(station.mac[3],HEX)+ ":" + String(station.mac[4],HEX)+ ":" + String(station.mac[5],HEX));
    }
});
#ifdef CYD
  // update results
  app.onRepeat(100, []() {
    if (displayOnToggle) {
      lv_task_handler();  // let the GUI do its work
      lv_tick_inc(100);     // tell LVGL how much time has passed should match app.onRepeat
      LVGLdataWindDebug();    
    }
    //if (demoModeToggle) {
    //  demoIncr();
    //}
  });
#endif
}

void loop() { 
#ifdef CYDXXX
  lv_task_handler();  // let the GUI do its work
  lv_tick_inc(10);     // tell LVGL how much time has passed
  delay(10);           // let this time pass
#endif
  app.tick(); 
}

#ifdef N2K
void demoInit() {
  n2k::windSpeedKnots = 9.8;
  n2k::windAngleDegrees = 47;
  n2k::rotateout = 30;
  n2k::mastIMUdeg = 100;
  n2k::boatIMUdeg = 120;
  mastAngle[1] = mastAngle[0] = n2k::mastIMUdeg - n2k::boatIMUdeg;
  BoatData.TrueHeading = 131;
}

void demoIncr() {
  n2k::windSpeedKnots -= 1;
  if (n2k::windSpeedKnots < 0) n2k::windSpeedKnots = 10;
  n2k::windAngleDegrees -=5;
  if (n2k::windAngleDegrees < 0) n2k::windAngleDegrees = 100;
  n2k::rotateout -=3;
  if (n2k::rotateout < 0) n2k::rotateout = 45;
  n2k::mastIMUdeg -=10 ;
  if (n2k::mastIMUdeg < 0) n2k::mastIMUdeg = 200;
  n2k::boatIMUdeg -= 9;
  if (n2k::boatIMUdeg < 0) n2k::boatIMUdeg = 220;
  BoatData.TrueHeading -= 10;
  if (BoatData.TrueHeading < 0) BoatData.TrueHeading = 231;
  mastAngle[0] = mastAngle[1] = n2k::mastIMUdeg - n2k::boatIMUdeg;
}
#endif
#endif