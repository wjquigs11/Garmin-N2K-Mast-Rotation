/* 
Integrated (single ESP32) mast rotation correction 
Relies on ESPberry to get RPI header for MCP2515-based HAT, since MCP2515 module causes kernel panic
Added PGN for "rudder" to output mast rotation angle from Honeywell sensor
Also reads NMEA0183 data from ICOM VHF and injects it onto N2K bus
Web server for displaying wind speed/angle/mast rotation
TBD: add wiring from TX on Serial(1) to Autohelm plug; port autopilot code here and modify web interface
TBD: translate apparent wind to Seatalk1 and send to tiller pilot
*/
#include "compass.h"
#include "windparse.h"
#include "BoatData.h"

// object-oriented classes
#include "BNO085Compass.h"
#include "logto.h"

#ifdef PICAN
#include <N2kMsg.h>
#include <NMEA2000.h>
#include <mcp_can.h>
#include <NMEA2000_mcp.h>
#endif
#ifdef RS485CAN
#include "mcp2515_can.h"
#endif

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

#ifdef D1MINI
#define CAN_TX_PIN GPIO_NUM_17
#define CAN_RX_PIN GPIO_NUM_16
#endif

#if defined(RS485CAN) || defined(OGDISPLAY)  // on SPI
#define OLED_MOSI  23 // RPI 19
#define OLED_CLK   18 // RPI 23
#define OLED_CS    14 // RPI pin 31
#define OLED_RESET 26 // RPI pin 16 - IO26
#define OLED_DC    13 // RPI pin 7 - IO13
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, OLED_DC, OLED_RESET, OLED_CS);
Adafruit_SSD1306 *display;
#endif

#define CAN_TX_PIN GPIO_NUM_27 // 27 = IO27, not GPIO27, RPI header pin 18
#define CAN_RX_PIN GPIO_NUM_35 // RPI header pin 15

#define N2k_SPI_CS_PIN 5    

#ifdef PICAN
tNMEA2000 *n2kWind;
bool n2kWindOpen = false;
Adafruit_BME280 bme;
bool bmeFound;
#endif

bool passThrough = false;

// display
// v1 uses Wire for I2C at standard pins, no defs needed?
//#define OGDISPLAY   // original controller is the only one with I2C display
#ifdef OGDISPLAYXXX  
#define OLED_RESET 4
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define SDA_PIN 16
#define SCL_PIN 17
Adafruit_SSD1306 *display;
#endif

#ifdef RS485CAN
// Define CAN Ports and their Chip Select/Enable
// RS485 CAN HAT
#define SPI_CS_PIN 5
mcp2515_can n2kWind(SPI_CS_PIN); // CAN0 interface CS
const int CAN0_INT = 25;    // RPi Pin 12 -- IO25#endif
#endif

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
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SSD1306 *display;
#endif // SH_ESP32

using namespace reactesp;
ReactESP app;
#define RECOVERY_RETRY_MS 1000  // How long to attempt CAN bus recovery
bool stackTrace = false;
TwoWire *i2c;

#if defined(NMEA0183)
#define NMEA0183serial Serial1
#define NMEA0183RX 16 // ESPberry RX0 = IO16? 
#define NMEA0183TX 15 
// SK Pang says it's mapped to /dev/ttyS0 which is 14(tx) and 15(rx) on RPI
// but that's RPI GPIOs, and RPI GPIO 15 is mapped to IO16 on ESPBerry
//#define SER_BUF_SIZE (unsigned int)(1024)
#define NMEA0183BAUD 4800
tNMEA0183 NMEA0183_3;
#endif

#if defined(RTK)
#define RTKserial Serial2
#define RTKRX 33
#define RTKTX 32
#define RTKBAUD 115200
tNMEA0183 RTKport;
#endif

tBoatData BoatData;
tRTKstats RTKdata;
tEnvStats ENVdata;

// TBD: I should add mag heading, mast heading etc (all the globals) to BoatData struct so I could have a single extern in each file

//Stream *read_stream = &Serial;
Stream *forward_stream = &Serial;

tNMEA2000 *n2kMain;
bool n2kMainOpen = false;
extern tN2kMsg correctN2kMsg;

#ifdef HONEY
// Honeywell sensor
extern movingAvg honeywellSensor;                // define the moving average object
extern float mastRotate;
extern int PotValue, PotLo, PotHi;
extern Adafruit_ADS1015 ads;
extern int adsInit;
#endif
extern float rotateout;

// timing/display
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

extern int num_0183_messages;
extern int num_0183_fail;
extern int num_0183_ok;

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
extern JSONVar readings;
void readPrefs();
bool initWiFi();
void startAP();
void startWebServer();
bool sendMastControl();
String getSensorReadings();

String host = "ESPwind";

// mast compass
int convertMagHeading(const tN2kMsg &N2kMsg); // magnetic heading of boat (from e.g. B&G compass)
float parseMastHeading(const tN2kMsg &N2kMsg);  // mast heading
float parseN2KHeading(const tN2kMsg &N2kMsg); // boat heading from external source
//float getCompass(int correction);      // boat heading from internal ESP32 compass
void httpInit(const char* serverName);
extern const char* serverName;
float mastOrientation; // delta between mast compass and boat compass
int sensOrientation; // delta between mast centered and Honeywell sensor reading at center
int boatOrientation; // delta between boat compass and magnetic north
int rtkOrientation;  // delta between boat RTK compass and TRUE north
float boatCompassTrue;
float mastCompassDeg;
float mastDelta;
extern tN2kWindReference wRef;
#ifdef BNO_GRV
movingAvg mastCompDelta(10);
#endif
void mastHeading();
int mastAngle[2]; // array for both sensors
// 0 = honeywell
// 1 = compass
float getMastHeading();

bool imuReady=false;
bool mastIMUready=false;

#ifdef BNO08X
BNO085Compass compass;
#endif

int headingErrCount;

// Time after which we should reboot if we haven't received any CAN messages
#define MAX_RX_WAIT_TIME_MS 30000

bool displayOnToggle=true, honeywellOnToggle=false, demoModeToggle=false;
//extern bool teleplot;

File consLog;

void WebSerialonMessage(uint8_t *data, size_t len);
//void log::log::toAll(String s);
//void logToAlln(String s);
void i2cScan(TwoWire Wire);
void demoIncr();
float readCompassDelta();
int compassDifference(int angle1, int angle2);

void ToggleLed() {
  static bool led_state = false;
  digitalWrite(LED_BUILTIN, led_state);
  led_state = !led_state;
}

#ifdef RS485CAN
void parseWindCAN();
#else
void ParseWindN2K(const tN2kMsg &N2kMsg);
void ParseCompassN2K(const tN2kMsg &N2kMsg);
#endif
void BoatSpeed(const tN2kMsg &N2kMsg);

const unsigned long TransmitMessages[] PROGMEM={130306L,127250L,0};

// for now, keep main bus as is, but I might swap to process depth on new controller on timo-managed bus
// NMEA 2000 message handler for main bus
void HandleNMEA2000MsgMain(const tN2kMsg &N2kMsg) {
  if (stackTrace) Serial.println("HandleNMEA2000MsgMain");
   
  //Serial.print("main: "); N2kMsg.Print(&Serial);
  n2kMainOpen = true;
  //n2kMain->SetForwardStream(forward_stream);
  num_n2k_messages++;
  time_since_last_can_rx = 0;
  ToggleLed();
  switch (N2kMsg.PGN) {
    case 127250L: // magnetic heading, use to set variation but do not override internal compass
      //Serial.printf("MAIN bus got Heading %d source %d\n", N2kMsg.PGN, N2kMsg.Source);
//      ParseCompassN2K(N2kMsg); // punting for now but this needs to be different
      break;
    case 130306L:
//      Serial.printf("MAIN Wind: %d", N2kMsg.PGN);
#ifdef SINGLECAN
#ifdef PICAN
      ParseWindN2K(N2kMsg);
#endif
#else
      //Serial.printf("alert: wind PGN on main bus");
      break;
#endif
    case 128259L:
//      Serial.printf("MAIN BoatSpeed: %d\n", N2kMsg.PGN);
      BoatSpeed(N2kMsg);
      break;
#ifdef REPORTRUDDER
    case 127245L: {
#ifdef DEBUG
      Serial.println(N2kMsg.PGN);
#endif
      // "rudder" (mast) angle
      double rudPos, angleOrder;
      unsigned char Instance;
      tN2kRudderDirectionOrder Direction;
      ParseN2kPGN127245(N2kMsg, rudPos, Instance, Direction, angleOrder);
      mastAngle[Instance] = (rudPos * 4068) / 71;   // cheap rads to degree
      break; }
#endif
  }
}

//#define WINDDIAG

void windCounter() {
  num_wind_messages++;
  total_time_since_last_wind += time_since_last_wind_rx;
  avg_time_since_last_wind = total_time_since_last_wind / num_wind_messages;
#ifdef WINDDIAG
  if (num_wind_messages % 100 == 0) {
    Serial.printf("last wind time: %2.2ld avg wind time: %2.2ld ms", time_since_last_wind_rx, avg_time_since_last_wind);
    if (time_since_last_wind_rx > 0.0)
      Serial.printf(" %2.2ld Hz", 1000.0/avg_time_since_last_wind);
    Serial.println();
  }
#endif
  time_since_last_wind_rx = 0;
}

//#define MASTCOMPDIAG

void mastCompCounter() {
  num_mastcomp_messages++;
  total_time_since_last_mastcomp += time_since_last_mastcomp_rx;
  avg_time_since_last_mastcomp = total_time_since_last_mastcomp / num_mastcomp_messages;
#ifdef MASTCOMPDIAG
  if (num_mastcomp_messages % 100 == 0) {
    Serial.printf("last mastcomp time: %2.2ld (secs) avg mastcomp time: %2.2ld ms", time_since_last_mastcomp_rx, avg_time_since_last_mastcomp);
    if (time_since_last_mastcomp_rx > 0.0)
      Serial.printf(" %2.2ld Hz", 1000.0/avg_time_since_last_mastcomp);
    Serial.println();
  }
#endif
  time_since_last_mastcomp_rx = 0;
}

#ifdef PICAN
// on the PICAN-M I was able to instantiate the MCP version of the Timo library, 
// so I use it to parse Wind and Heading from the mast

// NMEA 2000 message handler for wind bus: check if message is wind and handle it
void HandleNMEA2000MsgWind(const tN2kMsg &N2kMsg) {
  if (stackTrace) {
    //Serial.println("HandleNMEA2000MsgWind");
    N2kMsg.Print(&Serial);
    //Serial.printf("wind t: %d R: %d\n", millis(), N2kMsg.PGN);   
  }
  windCounter();
  if (!n2kWindOpen) {
    n2kWindOpen = true;
    //n2kWind->SetForwardStream(forward_stream);
  }
  ToggleLed();
  // problematic since mast compass can come back online
  //if (time_since_last_mastcomp_rx > 5000) // 5 second timeout on mast compass
  //  mastCompassDeg = -1;
  switch (N2kMsg.PGN) {
    case 130306L:
      ParseWindN2K(N2kMsg);
      break;
    case 128259L:
      BoatSpeed(N2kMsg);
      break; 
    case 127250L: 
      ParseCompassN2K(N2kMsg); // wind bus == mast compass = relative bearing
      break;  
    } 
}
#endif // PICAN

String can_state;

void RecoverFromCANBusOff() {
  if (stackTrace) Serial.println("RecoverFromCANBusOff");

  // This recovery routine first discussed in
  // https://www.esp32.com/viewtopic.php?t=5010 and also implemented in
  // https://github.com/wellenvogel/esp32-nmea2000
  static bool recovery_in_progress = false;
  static elapsedMillis recovery_timer;
  if (recovery_in_progress && recovery_timer < RECOVERY_RETRY_MS) {
    return;
  }
  recovery_in_progress = true;
  recovery_timer = 0;
  // Abort transmission
  MODULE_CAN->CMR.B.AT = 1;
  // read SR after write to CMR to settle register changes
  (void)MODULE_CAN->SR.U;

  // Reset error counters
  MODULE_CAN->TXERR.U = 127;
  MODULE_CAN->RXERR.U = 0;
  // Release Reset mode
  MODULE_CAN->MOD.B.RM = 0;
}

void PollCANStatus() {
  // CAN controller registers are SJA1000 compatible.
  // Bus status value 0 indicates bus-on; value 1 indicates bus-off.
  unsigned int bus_status = MODULE_CAN->SR.B.BS;

  switch (bus_status) {
    case 0:
      can_state = "RUN";
      break;
    case 1:
      can_state = "OFF";
      // try to automatically recover
      //RecoverFromCANBusOff();
      break;
  }
}

#ifdef DISPLAYON
void OLEDdataWindDebug() {      
  double windSpeedKnots = WindSensor::windSpeedKnots;
  double windAngleDegrees = WindSensor::windAngleDegrees;
  display->clearDisplay();
  display->setCursor(0, 5);
  //display->printf("CAN: %s ", can_state.c_str());
  unsigned long uptime = millis() / 1000;
  display->printf("Up:%*lu Wifi:", 3, uptime % 10000); // just print last 3 digits
  if (WiFi.status() == WL_CONNECTED)
    display->printf("%s\n", WiFi.SSID().substring(0,7));
  else display->printf("----\n");
  if (uptime % 60 == 0)  
    log::toAll("uptime: " + String(uptime) + " heap: " + String(ESP.getFreeHeap()));
  display->printf("N2K:%d ", num_n2k_messages % 10000);
  display->printf("Wind:%d\n", num_wind_messages % 10000);
  display->printf("S/A/R:%2.1f/%2.0f/%2.0f\n", windSpeedKnots, windAngleDegrees,rotateout);
  //sprintf(prbuf,"S/A/R:%2.1f/%2.0f/%2.0f\n", windSpeedKnots, windAngleDegrees,rotateout);
  //log::toAll(prbuf);
  //display->printf("Rot:%d\n", mastRotate);
#ifdef HONEY
  if (honeywellOnToggle) {
    display->printf("Sens:%d/%d/%d:%d\n", PotLo, PotValue, PotHi, mastAngle[0]);
  }
#endif
#ifdef BNO_GRV
  if (compass.OnToggle) {
    //display->printf("M:%.1f B:%.1f T:%0.1f\n", mastCompassDeg, compass.boatIMU, BoatData.trueHeading);
    display->printf("M:%.1f B:%.1f H:%.0f\n", mastCompassDeg, compass.boatIMU, compass.boatHeading);
    display->printf("Delta:%2d\n", mastAngle[1]);
  }
#endif
#ifdef MASTIMU
  display->printf("Mast:%2.0f/Rot:%2.0f\n",mastCompassDeg,mastRotate);
#endif
#ifdef BNO08X
  display->printf("M:%2.0f/T:%2.0f\n", compass.boatHeading,pBD->trueHeading);
#else
  display->printf("T:%2.0f\n", pBD->trueHeading);
#endif
  display->display();
}
#endif

void setup() {
  Serial.begin(115200);
  delay(300);
  log::toAll("starting wind correction");

  esp_log_level_set("*", ESP_LOG_VERBOSE);  // Set global log level
  //esp_task_wdt_init(timeout_seconds, true); // task timeout default 5 secs

  //adc1_config_width(ADC_WIDTH_BIT_12);
  //adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_MAX);

  Wire.begin();
  // slow down i2c to handle more devices
  //Wire.setClock(100000);

#if defined(RS485CAN) || defined(OGDISPLAY)  // on SPI
  Serial.println("reset OLED");
  pinMode(OLED_RESET, OUTPUT);  // RES Pin Display
  digitalWrite(OLED_RESET, LOW);
  delay(500);
  digitalWrite(OLED_RESET, HIGH);
  display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, OLED_DC, OLED_RESET, OLED_CS);
  if(!display->begin(SSD1306_EXTERNALVCC, 0, true))
    log::toAll(F("SSD1306 SPI allocation failed"));
  else
    log::toAll(F("SSD1306 SPI allocation success"));
#endif
#if 0 // original controller now uses SPI display
  Serial.println("reset OLED");
  pinMode(OLED_RESET, OUTPUT);  // RES Pin Display
  digitalWrite(OLED_RESET, LOW);
  delay(500);
  digitalWrite(OLED_RESET, HIGH);
  display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);  // wire1
  // SSD1306_EXTERNALVCC for 5V from Pi/ESPberry header
  if(!display->begin(SSD1306_EXTERNALVCC, SCREEN_ADDRESS, true, false))
    Serial.println(F("SSD1306 allocation failed"));
  else
    Serial.println(F("SSD1306 allocation success"));
#endif
#if defined(SH_ESP32) && defined(DISPLAYON)
  Serial.println("SH_ESP32 display");
  i2c = new TwoWire(0);
  if (i2c->begin(SDA_PIN, SCL_PIN)) {
    display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, i2c, -1);
    if(!display->begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
      log::toAll(F("SSD1306 I2C allocation failed"));
    else
      log::toAll(F("SSD1306 I2C allocation success"));
  }
#endif
#ifdef DISPLAYON
  log::toAll("OLED started");
  display->ssd1306_command(SSD1306_SETCONTRAST);
  display->ssd1306_command(0);  // dim display
  display->clearDisplay();
  display->setTextSize(1);             
  display->setTextColor(SSD1306_WHITE);
  display->setRotation(2);
  display->setCursor(10,10);
  display->println(F("\nESP32 Mast\nRotation\nCorrection"));
  display->display();
  log::toAll("display init");
#endif

  // toggle the LED pin at rate of 1 Hz
  pinMode(LED_BUILTIN, OUTPUT);
  app.onRepeatMicros(1e6 / 1, []() { ToggleLed(); });
#ifdef HONEY
#ifdef INA219
if (!ina219.begin()) {
  Serial.println("Failed to find INA219 chip");
  i2cScan(Wire);
#else
  if (!(adsInit = ads.begin())) {  // start ADS1015 ADC default 0x4A
    log::toAll("ads sensor not found");
    honeywellOnToggle = false;
    //i2cScan(Wire);
#endif // INA219
  } else {
    honeywellSensor.begin();    // Instantiates the moving average object
  }
#endif // HONEY
  pBD=&BoatData; // need this even if we're not using 0183
#ifdef RTK
  pRTK=&RTKdata;
#endif
  pENV=&ENVdata;
  // Set up NMEA0183 ports and handlers
#if defined(NMEA0183)
  NMEA0183_3.SetMsgHandler(HandleNMEA0183Msg);
  //DebugNMEA0183Handlers(&Serial);
  NMEA0183serial.begin(NMEA0183BAUD, SERIAL_8N1, NMEA0183RX, NMEA0183TX);
  NMEA0183_3.SetMessageStream(&NMEA0183serial);
  NMEA0183_3.Open();
  if (!NMEA0183serial) 
    log::toAll("failed to open NMEA0183 serial port");
  else
    log::toAll("opened NMEA0183 serial port");
#endif
#ifdef RTK
  RTKport.SetMsgHandler(HandleNMEA0183Msg);
  RTKserial.begin(RTKBAUD,SERIAL_8N1,RTKRX,RTKTX);
  RTKport.SetMessageStream(&RTKserial);
  RTKport.Open();
  if (!RTKserial)
    log::toAll("failed to open RTK serial port");
  else
    log::toAll("opened RTK serial port");
#endif
#ifdef N2K
  // instantiate the NMEA2000 object for the main bus
  n2kMain = new tNMEA2000_esp32(CAN_TX_PIN, CAN_RX_PIN);
  // Reserve enough buffer for sending all messages.
  n2kMain->SetN2kCANSendFrameBufSize(250);
  n2kMain->SetN2kCANReceiveFrameBufSize(250);
  // Set Product information
  n2kMain->SetProductInformation(
      "20240608",  // Manufacturer's Model serial code (max 32 chars)
      103,         // Manufacturer's product code
      "SH-ESP32 Wind Correction",  // Manufacturer's Model ID (max 33 chars)
      "0.1.0.0 (2021-03-31)",  // Manufacturer's Software version code (max 40
                               // chars)
      "0.0.3.1 (2021-03-07)"   // Manufacturer's Model version (max 24 chars)
  );
  // Set device information
  n2kMain->SetDeviceInformation(
      20240608, // serial
      132, // analog gateway
      25, // inter/intranetwork device 
      2046
  );
  n2kMain->SetMode(tNMEA2000::N2km_ListenAndSend);
  n2kMain->SetForwardType(tNMEA2000::fwdt_Text); // Show bus data in clear
  //n2kMain->EnableForward(false);
  n2kMain->SetForwardOwnMessages(false);
  n2kMain->SetMsgHandler(HandleNMEA2000MsgMain); 
  n2kMain->ExtendTransmitMessages(TransmitMessages);
  if (n2kMain->Open()) {
    log::toAll("opening n2kMain");
  } else {
    log::toAll("failed to open n2kMain");
  }
#endif
#if defined(PICAN) 
  // instantiate the NMEA2000 object for the wind bus
  // does not need to register since it's only listening
  n2kWind = new tNMEA2000_mcp(N2k_SPI_CS_PIN,MCP_16MHz);
  //n2kWind = new tNMEA2000_esp32(CAN_TX_PIN, CAN_RX_PIN);
  n2kWind->SetN2kCANMsgBufSize(8);
  n2kWind->SetN2kCANReceiveFrameBufSize(100);
  n2kWind->SetMode(tNMEA2000::N2km_ListenAndSend);
  n2kWind->SetForwardType(tNMEA2000::fwdt_Text);
  //n2kWind->EnableForward(false); 
  n2kWind->SetForwardOwnMessages(true);
  n2kWind->SetMsgHandler(HandleNMEA2000MsgWind);
  if (n2kWind->Open()) {
    log::toAll("opening n2kWind");
  } else {
    log::toAll("failed to open n2kWind");
  }
  // PICAN has BME280 environmental sensor
  if (bmeFound = bme.begin(0x76)) {
    Serial.println("BME280 found");
  } else {
    Serial.println("Could not find a valid BME280 sensor.");
    //i2cScan(Wire);
  }
#elif defined(RS485CAN)
  // Initialize the CAN port
    //n2kWind.reset();
    //int cRetCode;
    //if ((n2kWind.setBitrate(CAN_250KBPS, MCP_16MHZ) == CAN_OK) && (cRetCode = n2kWind.setListenOnlyMode() == CAN_OK))
  if (n2kWind.begin(CAN_250KBPS, MCP_12MHz) == CAN_OK)
        log::toAll("CAN0 (wind) Initialized.");
    else {
        log::toAll("CAN0 (wind) Initialization Error.");
    }
#endif
 
  if (!SPIFFS.begin())
    log::toAll("An error has occurred while mounting SPIFFS");
  else
    log::toAll("SPIFFS mounted successfully");

  // start a console.log file in case we crash before Webserial starts
  if (SPIFFS.exists("/console.log")) {
    SPIFFS.remove("/console.log");
  }
  consLog = SPIFFS.open("/console.log", "w", true);
  if (!consLog) {
    log::toAll("failed to open console log");
  }
  if (consLog.println("ESP compass console log.")) {
    log::toAll("console log written");
  } else {
    log::toAll("console log write failed");
  }

  readPrefs();
#ifdef GPX
  initGPX();
#endif
#ifdef WINDLOG
  initWindLog();
#endif

// making it async
  if (initWiFi()) {
    startWebServer();
  } else {
    startAP();
  }
  serverStarted=true;
  log::toAll("ESP local MAC addr: " + WiFi.macAddress());

#ifdef ICM209
  imu.begin(Wire1, ICM209);
  imuReady = (imu.status == ICM_20948_Stat_Ok);
  if (imuReady) {
    xTaskCreatePinnedToCore(
      getICMheading, // Task function
      "getICMheading",      // Name of task
      2048,        // Stack size (bytes)
      NULL,         // Parameter to pass to the function
      1,            // Priority of the task
      &compassTask,        // Task handle
      0
    );
    log::toAll("ICM20948 found.");
    avgYaw.begin();
  } else {
    log::toAll("Failed to find ICM20948");
    //i2cScan(Wire1);
  }
#endif
#ifdef ISM330
  imuReady = setupISM330(Wire1, ISM330);
  if (imuReady) {
    xTaskCreatePinnedToCore(
      getISMheading, // Task function
      "getISMHeading",      // Name of task
      2048,        // Stack size (bytes)
      NULL,         // Parameter to pass to the function
      1,            // Priority of the task
      &compassTask,        // Task handle
      0
    );
    log::toAll("IMU ISM330 detected.");  
  } else {
    log::toAll("Failed to find IMU ISM330 @ 0x" + String(ISM330,HEX));
    //i2cScan(Wire);
  }
#endif
#ifdef BNO08X
  log::toAll("BNO08X");
  imuReady = compass.OnToggle = compass.begin();
  pBD->Variation = VARIATION;
  if (imuReady) {
    //compass.setReports();
  } else {
    log::toAll("BNO08x not found");
    //i2cScan(Wire);
  }
#endif
#ifdef BNO_GRV
  mastCompDelta.begin();
#endif

  ElegantOTA.begin(&server);
  WebSerial.setBuffer(128);
  WebSerial.begin(&server);
  WebSerial.setBuffer(100);
  WebSerial.onMessage(WebSerialonMessage);

#if 0
  if (WiFi.status() == WL_CONNECTED) {
    // Update the time
    //#define RETRIES 10
    //int count=0;
    //if (!timeClient.update() && count++ < RETRIES) {
        timeClient.update();
    //}
    log::toAll(timeClient.getFormattedDate());
  }
#endif

  log::toAll("ESP flash size 0x" + String(ESP.getFlashChipSize(),HEX)); // 4194304

  consLog.flush();

#ifdef N2K
  // No need to parse the messages at every single loop iteration; 1 ms will do
    app.onRepeat(1, []() {
    PollCANStatus();
    n2kMain->ParseMessages();
#if defined(RS485CAN)
    parseWindCAN();
#endif
#if defined(PICAN)
    n2kWind->ParseMessages();
#endif
#if defined(NMEA0183)
    NMEA0183_3.ParseMessages(); // GPS from ICOM
#ifdef DEBUG_0183
    tNMEA0183Msg NMEA0183Msg;
    while (NMEA0183_3.GetMessage(NMEA0183Msg)) {
    Serial.print(NMEA0183Msg.Sender());
    Serial.print(NMEA0183Msg.MessageCode()); Serial.print(" ");
    for (int i=0; i < NMEA0183Msg.FieldCount(); i++) {
      Serial.print(NMEA0183Msg.Field(i));
      if ( i<NMEA0183Msg.FieldCount()-1 ) Serial.print(" ");
    }
   Serial.print("\n");
  }
#endif
#endif
#ifdef RTK
#ifdef DEBUG_RTK
    tNMEA0183Msg NMEA0183Msg;
    while (RTKport.GetMessage(NMEA0183Msg)) {
    Serial.print(NMEA0183Msg.Sender());
    Serial.print(NMEA0183Msg.MessageCode()); Serial.print(" ");
    for (int i=0; i < NMEA0183Msg.FieldCount(); i++) {
      Serial.print(NMEA0183Msg.Field(i));
      if ( i<NMEA0183Msg.FieldCount()-1 ) Serial.print(" ");
    }
   Serial.print("\n");
  }
#else
  RTKport.ParseMessages();  // GPS etc from WITmotion/UM982
#endif
#endif
  });
#endif

  // update web page
  app.onRepeat(WebTimerDelay, []() {
    static int counter;
    //log::toAll("transmit sensor readings");
    // Send Events to the client with the Sensor Readings Every x seconds
    //events.send("ping",NULL,millis());
    events.send(getSensorReadings().c_str(),"new_readings" ,millis());
//    mastCompassDeg = getMastHeading();
    // NTP client appears to hang (synchronous)
    //if (counter++ % 600 == 0) {
    //  log::toAll(timeClient.getFormattedDate());
#ifdef PICANXXX
    if (bmeFound) {
      log::toAll("Temperature = " + String(1.8 * bme.readTemperature() + 32));
      int press = bme.readPressure();
      log::toAll("Pressure = " + String(press / 100.0F) + " hPa " + String(press * 0.0002953) + " inHg");
      //log::toAll("Altitude = " + String(bme.readAltitude(SEALEVELPRESSURE_HPA)*3.28084));
      log::toAll("Humidity = " + String(bme.readHumidity()) + " %");
    }
#endif
    consLog.flush();
  });

#ifdef GPX
  static int trackPtCounter=0;
  app.onRepeat(GPXTIMER, []() {
    writeTrackPoint(GPXlogFile, pBD->Latitude, pBD->Longitude, pBD->SOG, pBD->trueHeading, pBD->GPSTime);
    if (trackPtCounter++ % 500 == 0) {
      writeHeadingWaypoint(WPTlogFile, pBD->Latitude, pBD->Longitude, pBD->SOG, pBD->trueHeading, pBD->GPSTime);
      sprintf(prbuf, "lat: %3.4lf lon: %3.4f SOG: %2.2f Heading: %2.2f", pBD->Latitude, pBD->Longitude, pBD->SOG, pBD->trueHeading);
      log::toAll(prbuf);
    }
  });
#endif

#ifdef WINDLOG
  app.onRepeat(WINDTIMER, []() {
    if (windLogging)
      writeWindPoint(windLogFile, millis()/1000, rotateout, WindSensor::windSpeedKnots, pBD->STW, pBD->TWA, pBD->TWS, pBD->TWD, pBD->VMG, pBD->trueHeading);
  });
#endif

// if wifi not connected, we're only going to attempt reconnect once every 5 minutes
// if we get in range, it's simpler to reboot than to constantly check
// the first time we call this, wifi will not be connected...trying "async" connection to make reboot faster
#if 1
// connecting to wifi seems to be hanging output
  app.onRepeat(300000, []() {
    // we have to clear counters sometime because otherwise they roll off screen
    //num_n2k_messages = 0;
    //num_wind_messages = 0;  
    if (WiFi.status() != WL_CONNECTED) {
      log::toAll("WiFi not connected");
      initWiFi();
      if (WiFi.status() == WL_CONNECTED) {
        startWebServer();
        serverStarted=true;
        timeClient.update();
        log::toAll(timeClient.getFormattedDate());
      }
    }
    // check for connected clients
    int numClients = WiFi.softAPgetStationNum();
    log::toAll("Number of connected clients: " + String(numClients));
    wifi_sta_list_t stationList;
    esp_wifi_ap_get_sta_list(&stationList);
    for (int i = 0; i < stationList.num; i++) {
      wifi_sta_info_t station = stationList.sta[i];
      log::toAll("Client " + String(i+1) + " MAC: " + String(station.mac[0],HEX) + ":" + String(station.mac[1],HEX)+ ":" + String(station.mac[2],HEX)+ ":" + String(station.mac[3],HEX)+ ":" + String(station.mac[4],HEX)+ ":" + String(station.mac[5],HEX));
    }
});
#endif

#ifdef BNO08X
  if (imuReady) {
    log::toAll("checking heading at " + String(compass.frequency));
    app.onRepeat(compass.frequency, []() {
      float heading;
      int err;
      if ((err = compass.getHeading(boatOrientation)) < 0) {
        headingErrCount++;
        if (headingErrCount % 500 == 0)
          log::toAll("heading error count: " + String(headingErrCount) + "ret val: " + String(err));
      } else
        pBD->magHeading = compass.boatHeading;
    });
  }
  // check in for new heading 100 msecs = 10Hz
  if (imuReady && !demoModeToggle)
    app.onRepeat(compass.frequency, []() {
      // TBD: delete reaction and recreate if frequency changes
      // Heading, corrected for local variation (acquired from ICOM via NMEA0183)
      // TBD: set Variation if we get a Heading PGN on main bus that includes it
    if (compass.teleplot) {
        Serial.print(">mast:");
        float corrMastComp = mastCompassDeg+mastOrientation;
        if (corrMastComp > 359) corrMastComp -= 360;
        if (corrMastComp < 0) corrMastComp += 360;
        Serial.println(corrMastComp);
        Serial.print(">boatIMU:");
        Serial.println(compass.boatIMU);
        Serial.print(">delta:");
        Serial.println(mastDelta);  
        Serial.printf(">boatCOMP:%0.2f\n", compass.boatHeading);
        Serial.printf(">true:%0.2f\n",BoatData.trueHeading); 
        Serial.printf(">calstat:%d\n", compass.boatCalStatus);
        Serial.printf(">acc:%0.2f\n", compass.boatAccuracy*RADTODEG);
    }
#if defined(N2K) && !defined(RTK)
    // transmit heading 
    pBD->trueHeading = compass.boatHeading + pBD->Variation;
    if (n2kMainOpen) {
      SetN2kPGN127250(correctN2kMsg, 0xFF, (double)compass.boatHeading*DEGTORAD, N2kDoubleNA, N2kDoubleNA, N2khr_magnetic);
      if (!n2kMain->SendMsg(correctN2kMsg)) {
        //log::toAll("Failed to send mag heading from compass reaction");
        num_wind_other_fail++;
      }
      SetN2kPGN127250(correctN2kMsg, 0xFF, (double)pBD->trueHeading*DEGTORAD, N2kDoubleNA, N2kDoubleNA, N2khr_true);
      if (!n2kMain->SendMsg(correctN2kMsg)) {
        //log::toAll("Failed to send true heading from compass reaction");
        num_wind_other_fail++;
      }
    }
#endif
    });
    else log::log::toAll("compass not ready no heading reaction");
#endif

#ifdef DISPLAYON
  // update results
  app.onRepeat(500, []() {
    if (displayOnToggle) OLEDdataWindDebug();
      else {
        display->clearDisplay();
        display->display();
      }
#ifdef DEMO
    if (demoModeToggle) {
      demoIncr();
    }
#endif
  });
  #endif // DISPLAYON

#if 1
  app.onRepeat(500, []() {
    calcTrueWindAngle();
    /*
    log::toAll("STW: " + String(pBD->STW*MTOKTS));
    log::toAll("AWS: " + String(WindSensor::windSpeedKnots) + " AWA: " + String(WindSensor::windAngleDegrees));
    log::toAll("TWS: " + String(pBD->TWS*MTOKTS) + " TWA: " + String(pBD->TWA));
    log::toAll("VMG: " + String(pBD->VMG*MTOKTS));
    log::toAll("maxTWS: " + String(pBD->maxTWS*MTOKTS));
    */
  });
#endif
}

#ifdef RTK
bool rtkDebug = false;
bool tuning = false;
const int MAXL = 256;
char keyBuffer[MAXL], gpsBuffer[MAXL];
int keyBufIdx, gpsBufIdx;
#endif

void loop() { 
  app.tick(); 
  WebSerial.loop();
  ElegantOTA.loop();
#ifdef RTK
  if (rtkDebug) {
    char incomingChar;
    while (Serial.available() > 0) {
      incomingChar = Serial.read();
      if (incomingChar == '\n' || incomingChar == '\r') {
        keyBuffer[keyBufIdx] = '\0'; // Null terminate
        // send to GPS
        //Serial.printf("command: %s\n", keyBuffer);
        RTKserial.println(keyBuffer);
        keyBufIdx = 0; // Reset for next line
      } else if (keyBufIdx < MAXL - 1) {
          keyBuffer[keyBufIdx++] = incomingChar;
      }
      //Serial.print(incomingChar);
    }  while (RTKserial.available() > 0) {
      incomingChar = RTKserial.read();
      if (incomingChar == '\n' || incomingChar == '\r') {
        gpsBuffer[gpsBufIdx] = '\0'; // Null terminate
        //processBuffer();
        Serial.printf("gps: %s\n", gpsBuffer);
        gpsBufIdx = 0; // Reset for next line
      } else if (gpsBufIdx < MAXL - 1) {
          gpsBuffer[gpsBufIdx++] = incomingChar;
      }
      //Serial.print(incomingChar);
    }
  }
#endif
}

#ifdef DEMO
void demoInit() {
  WindSensor::windSpeedKnots = 9.8;
  WindSensor::windAngleDegrees = 47;
  rotateout = 30;
  mastCompassDeg = 100;
  compass.boatIMU = 120;
  mastAngle[1] = mastAngle[0] = mastCompassDeg - compass.boatIMU;
#ifdef HONEY
  PotValue = 50;
#endif
  BoatData.trueHeading = 131;
}

void demoIncr() {
  WindSensor::windSpeedKnots -= 1;
  if (WindSensor::windSpeedKnots < 0) WindSensor::windSpeedKnots = 10;
  WindSensor::windAngleDegrees -=5;
  if (WindSensor::windAngleDegrees < 0) WindSensor::windAngleDegrees = 100;
  rotateout -=3;
  if (rotateout < 0) rotateout = 45;
  mastCompassDeg -=10 ;
  if (mastCompassDeg < 0) mastCompassDeg = 200;
  compass.boatIMU -= 9;
  if (compass.boatIMU < 0) compass.boatIMU = 220;
  BoatData.trueHeading -= 10;
  if (BoatData.trueHeading < 0) BoatData.trueHeading = 231;
  mastAngle[0] = mastAngle[1] = mastCompassDeg - compass.boatIMU;
#ifdef HONEY
  PotValue -= 5;
  if (PotValue < 0) PotValue = 100;
#endif
}
#endif
