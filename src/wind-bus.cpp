/* 
Integrated (single ESP32) mast rotation correction 
Relies on ESPberry to get RPI header for MCP2515-based HAT, since MCP2515 module causes kernel panic
Added PGN for "rudder" to output mast rotation angle from Honeywell sensor
Also reads NMEA0183 data from ICOM VHF and injects it onto N2K bus
Web server for displaying wind speed/angle/mast rotation
TBD: add wiring from TX on Serial(1) to Autohelm plug; port autopilot code here and modify web interface
TBD: translate apparent wind to Seatalk1 and send to tiller pilot
branch manual-parse-wind-bus to switch wind bus to not use Timo mcp library, since it's not working
*/
#include <Arduino.h>
#include <ActisenseReader.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <N2kMessages.h>
#include <NMEA2000_esp32.h>
#include <NMEA0183Msg.h>
#include <NMEA0183Messages.h>
#include "NMEA0183Handlers.h"
#include "BoatData.h"
#include <ReactESP.h>
#include <Wire.h>
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>
#include <movingAvg.h>
#include "elapsedMillis.h"
#include <Arduino.h>
#include <N2kMessages.h>
#include <Adafruit_ADS1X15.h>
#include <WiFi.h>
#include "SPIFFS.h"
#include <Arduino_JSON.h>
#include <ESPmDNS.h>
#include "mcp2515.h"
#include "can.h"
#include "Async_ConfigOnDoubleReset_Multi.h"
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include <WebSerial.h>
#include <Adafruit_BNO08x.h>
#include <HTTPClient.h>

#include "windparse.h"

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

#ifdef ESPBERRY
#define CAN_TX_PIN GPIO_NUM_27 // 27 = IO27, not GPIO27, RPI header pin 18
#define CAN_RX_PIN GPIO_NUM_35 // RPI header pin 15
//#define N2k_SPI_CS_PIN 5    
//#define N2k_CAN_INT_PIN 26
// for CMPS14
#define SDA_PIN 21
#define SCL_PIN 22
// for display
#define OLED_MOSI  23
#define OLED_CLK   18
#define OLED_CS    14 // RPI pin 31
#define OLED_RESET 25 // RPI pin 12 - IO25
#define OLED_DC    13 // RPI pin 7 - IO13
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, OLED_DC, OLED_RESET, OLED_CS);
#endif // ESPBERRY

// Define CAN Ports and their Chip Select/Enable
//#define CAN_RX_INTERRUPT_MODE 0
MCP2515 n2kWind(5); // CAN0 interface CS
const int CAN0_INT = 26;    // RPi Pin 16 - IO26

#ifdef PICANM  // v1 of the controller uses PICAN-M HAT
#define CAN_TX_PIN GPIO_NUM_27 // 27 = IO27, not GPIO27, RPI header pin 18
#define CAN_RX_PIN GPIO_NUM_35
// commenting out because I'm trying raw CAN parse
//#define N2k_SPI_CS_PIN 5    
//#define N2k_CAN_INT_PIN 22   
//tNMEA2000 *n2kWind;
// display
// v1 uses Wire for I2C at standard pins, no defs needed
#define OLED_RESET 4
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#endif

#ifdef SH_ESP32
//#define CAN_TX_PIN GPIO_NUM_32
//#define CAN_RX_PIN GPIO_NUM_34
// external transceiver because on-board one isn't working
#define CAN_RX_PIN GPIO_NUM_27
#define CAN_TX_PIN GPIO_NUM_25
//#define N2k_SPI_CS_PIN 5    
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
#endif // SH_ESP32

using namespace reactesp;

ReactESP app;

#define RECOVERY_RETRY_MS 1000  // How long to attempt CAN bus recovery

TwoWire *i2c;

#ifdef PICANM
#define NMEA0183serial Serial1
#define NMEA0183RX 16 // ESPberry RX0 = IO16? 
// SK Pang says it's mapped to /dev/ttyS0 which is 14(tx) and 15(rx) on RPI
// but that's RPI GPIOs, and RPI GPIO 15 is mapped to IO16 on ESPBerry
//#define SER_BUF_SIZE (unsigned int)(1024)
#define NMEA0183BAUD 4800
tNMEA0183 NMEA0183_3;
#endif
extern tBoatData *pBD;
tBoatData BoatData;
// TBD: I should add mag heading, mast heading etc (all the globals) to BoatData struct so I could have a single extern in each file

//Stream *read_stream = &Serial;
Stream *forward_stream = &Serial;

tNMEA2000 *n2kMain;

// Honeywell sensor
extern movingAvg honeywellSensor;                // define the moving average object
extern int mastRotate, rotateout;
extern int PotValue, PotLo, PotHi;
extern Adafruit_ADS1015 ads;
extern int adsInit;

// timing/display
int num_n2k_messages = 0;
int num_wind_messages = 0;
elapsedMillis time_since_last_can_rx = 0;
elapsedMillis time_since_last_wind_rx = 0;
unsigned long total_time_since_last_wind = 0.0;
unsigned long avg_time_since_last_wind = 0.0;

// defs for wifi
void initWebSocket();
//void notifyClients(String);
extern AsyncWebServer server;
bool serverStarted=false;
extern char *hostname;
extern int WebTimerDelay;
//extern AsyncWebSocket ws;
extern AsyncEventSource events;
extern JSONVar readings;
extern void setupWifi();
//extern void loopWifi();
void startWebServer();
String getSensorReadings();
void setupESPNOW();
bool sendMastControl();
extern DoubleResetDetector* drd;
extern void check_status();

String host = "ESPwind";

// mast compass
int convertMagHeading(const tN2kMsg &N2kMsg); // magnetic heading of boat (from e.g. B&G compass)
float parseMastHeading(const tN2kMsg &N2kMsg);  // mast heading
float parseN2KHeading(const tN2kMsg &N2kMsg); // boat heading from external source
float getCompass(int correction);      // boat heading from internal ESP32 compass
void httpInit(const char* serverName);
extern const char* serverName;
extern int mastOrientation;   // delta between mast compass and boat compass
extern float boatCompassDeg; // magnetic heading not corrected for variation
extern float mastCompassDeg;
extern float mastDelta;
void mastHeading();
float mastAngle[2]; // array for both sensors
// 0 = honeywell
// 1 = compass

//IMUType compassType = NONE; 
#ifdef CMPS14
// robotshop CMPS14 (boat compass)
IMUType compassType = CMPS14;
#endif
#ifdef BNO08X
// compass
const int CMPS14_ADDRESS = 0x60;
const int ADABNO = 0x4A;
const int BNO08X_RESET = -1;
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
IMUType compassType = BNO085;
#endif
bool compassReady=false;

// Time after which we should reboot if we haven't received any CAN messages
#define MAX_RX_WAIT_TIME_MS 30000

bool displayOnToggle=true, compassOnToggle=false, honeywellOnToggle=false;

void ToggleLed() {
  static bool led_state = false;
  digitalWrite(LED_BUILTIN, led_state);
  led_state = !led_state;
}

#ifdef ESPBERRY
void WindSpeed();
#else
void WindSpeed(const tN2kMsg &N2kMsg);
#endif
void BoatSpeed(const tN2kMsg &N2kMsg);

const unsigned long TransmitMessages[] PROGMEM={130306L,127250L,0};

// NMEA 2000 message handler for main bus
void HandleNMEA2000MsgMain(const tN2kMsg &N2kMsg) {   
  //Serial.print("main: "); N2kMsg.Print(&Serial);
  num_n2k_messages++;
  time_since_last_can_rx = 0;
  ToggleLed();
  switch (N2kMsg.PGN) {
    case 127250L: // magnetic heading, use to set variation but do not override internal compass
#ifdef DEBUG
      Serial.println(N2kMsg.PGN);
#endif
      unsigned char SID;
      double heading;
      double deviation;
      double variation;
      tN2kHeadingReference headingRef;
      // we have boat compass; just update variation from other compass
      if (ParseN2kPGN127250(N2kMsg, SID, heading, deviation, variation, headingRef)) {
        if (compassReady) {
            BoatData.Variation = variation;
        } else {
          // no boat compass, use external heading info
          if (compassOnToggle) {
            // got bearing; calculate mast heading
            // TBD: this isn't correct
            tN2kMsg correctN2kMsg;
            int mastRotate = convertMagHeading(N2kMsg);
            if (mastRotate > -1) {
              // for now (until you dive into SensESP), send rotation angle as "rate of turn"
              SetN2kPGN127251(correctN2kMsg,0xff,(mastRotate+50)*(M_PI/180));
              n2kMain->SendMsg(correctN2kMsg);
            }
          } 
          // !compassOnToggle (no internal compass), so set boat heading here
          // TBD: check if we're getting true or magnetic
          BoatData.TrueHeading = heading;
          BoatData.Variation = variation;
        } // else
      }
      break;
    case 130306L:
#ifdef SINGLECAN
      WindSpeed(N2kMsg);
#else
      //Serial.printf("alert: wind PGN on main bus\n");
      break;
#endif
    case 128259L:
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

#ifdef PICANMX // try to parse raw CAN bus
// This is problematic. If you keep the second bus on the original controller, 
// you need to split the wind parse into several pieces
// because now it's parsing manually. PITA!

// NMEA 2000 message handler for wind bus: check if message is wind and handle it
void HandleNMEA2000MsgWind(const tN2kMsg &N2kMsg) {   
  //N2kMsg.Print(&Serial);
  //Serial.printf("t: %d R: %d\n", millis(), N2kMsg.PGN);
  num_wind_messages++;
  time_since_last_can_rx = 0;
  ToggleLed();
  switch (N2kMsg.PGN) {
    case 130306L:
//#define WINDEBUG
#ifdef WINDEBUG
      total_time_since_last_wind += time_since_last_wind_rx;
      avg_time_since_last_wind = total_time_since_last_wind / num_wind_messages;
      if (num_wind_messages % 100 == 0) {
        Serial.printf("last wind time: %2.2ld avg wind time: %2.2ld ms", time_since_last_wind_rx, avg_time_since_last_wind);
        if (time_since_last_wind_rx > 0.0)
          Serial.printf(" %2.2ld Hz", 1000.0/avg_time_since_last_wind);
        Serial.println();
      }
      time_since_last_wind_rx = 0;
#endif
      WindSpeed(N2kMsg);
      break;
    case 128259L:
      BoatSpeed(N2kMsg);
      break; 
  } 
}
#endif

String can_state;

void RecoverFromCANBusOff() {
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
      can_state = "RUNNING";
      break;
    case 1:
      can_state = "BUS-OFF";
      // try to automatically recover
      //RecoverFromCANBusOff();
      break;
  }
}

//void OLEDdataWindDebug(int mastrotate, int rotateout) {                           
void OLEDdataWindDebug() {      
  double windSpeedKnots = WindSensor::windSpeedKnots;
  double windAngleDegrees = WindSensor::windAngleDegrees;
  display.clearDisplay();
  display.setCursor(0, 5);
  display.printf("CAN: %s ", can_state.c_str());
  display.printf("Up: %lu\n", millis() / 1000);
  display.printf("N2K: %d ", num_n2k_messages);
  display.printf("Wind: %d\n", num_wind_messages);
  display.printf("S/A/R:%2.1f/%2.1f/%2.1f\n", windSpeedKnots, windAngleDegrees,rotateout);
  //display.printf("Rot:%d\n", mastRotate);
  if (honeywellOnToggle) {
    display.printf("Sensor: %d/%d/%d\n", PotLo, PotValue, PotHi);
    display.printf("Angle: %d\n", mastAngle[0]);
  }
  if (compassOnToggle) {
    #ifdef MASTCOMPASS
    display.printf("M: %.1f B: %.1f\n", mastCompassDeg, boatCompassDeg);
    display.printf("Delta: %d\n", mastAngle[1]);
    #else
    display.printf("Heading: %.1f\n", boatCompassDeg);
    #endif
  }
  //Serial.printf("Hon: %d, Mag: %d\n", mastAngle[0], mastAngle[1]);
  display.display();
}

void WebSerialonMessage(uint8_t *data, size_t len);
void logToAll(String s);
void logToAlln(String s);
void i2cScan();
void ParseWindCAN();

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("starting wind correction");

  //adc1_config_width(ADC_WIDTH_BIT_12);
  //adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_MAX);

  Wire.begin();
  
#if !defined(SH_ESP32)
  pinMode(OLED_RESET, OUTPUT);  // RES Pin Display
  digitalWrite(OLED_RESET, LOW);
  delay(500);
  digitalWrite(OLED_RESET, HIGH);
#endif
#ifdef ESPBERRY
  if(!display.begin(SSD1306_EXTERNALVCC))
#endif
#ifdef PICANM
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS, true))
#endif
    Serial.println(F("SSD1306 allocation failed"));
  else
    Serial.println(F("SSD1306 allocation success"));
#ifdef SH_ESP32
  i2c = new TwoWire(0);
  i2c->begin(SDA_PIN, SCL_PIN);
  displayPtr = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, i2c, -1);
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
    Serial.println(F("SSD1306 allocation failed"));
  else
    Serial.println(F("SSD1306 allocation success"));
#endif

  display.clearDisplay();
  display.setTextSize(1);             
  display.setTextColor(SSD1306_WHITE);
  display.setRotation(2);
  display.setCursor(0,0);
  display.println(F("ESP32 Mast\nRotation\nCorrection"));

  // toggle the LED pin at rate of 1 Hz
  pinMode(LED_BUILTIN, OUTPUT);
  app.onRepeatMicros(1e6 / 1, []() { ToggleLed(); });
  if (!(adsInit = ads.begin())) {  // start ADS1015 ADC default 0x4A
    Serial.println("ads sensor not found");
    honeywellOnToggle = false;
  } else {
    Serial.println("init ADS");
    honeywellSensor.begin();    // Instantiates the moving average object
  }
  // Set up NMEA0183 ports and handlers
  pBD=&BoatData;
#ifdef PICANM
  NMEA0183_3.SetMsgHandler(HandleNMEA0183Msg);
  //DebugNMEA0183Handlers(&Serial);
  NMEA0183serial.begin(NMEA0183BAUD, SERIAL_8N1, NMEA0183RX, -1);
  //Serial1.begin(NMEA0183BAUD, SERIAL_8N1, NMEA0183RX, -1);
  NMEA0183_3.SetMessageStream(&NMEA0183serial);
  NMEA0183_3.Open();
  if (!NMEA0183serial) 
    Serial.println("failed to open NMEA0183 serial port");
  else
    Serial.println("opened NMEA0183 serial port");
#endif
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
  //n2kMain->SetForwardStream(forward_stream);
  n2kMain->SetMode(tNMEA2000::N2km_ListenAndSend);
  n2kMain->SetForwardType(tNMEA2000::fwdt_Text); // Show bus data in clear
  //n2kMain->EnableForward(true);
  //n2kMain->SetForwardOwnMessages(true);
  n2kMain->SetMsgHandler(HandleNMEA2000MsgMain); 
  n2kMain->ExtendTransmitMessages(TransmitMessages);
  Serial.println("opening n2kMain");
  n2kMain->Open();

#if defined(PICANMX) 
  // instantiate the NMEA2000 object for the wind bus
  // does not need to register since it's only listening
  //n2kWind = new tNMEA2000_mcp(N2k_SPI_CS_PIN,MCP_16MHz);
  n2kWind = new tNMEA2000_esp32(CAN_TX_PIN, CAN_RX_PIN);
  n2kWind->SetN2kCANMsgBufSize(8);
  n2kWind->SetN2kCANReceiveFrameBufSize(100);
  n2kWind->SetMode(tNMEA2000::N2km_ListenAndSend);
  //n2kWind->SetForwardStream(forward_stream);
  n2kWind->SetForwardType(tNMEA2000::fwdt_Text);
  n2kWind->EnableForward(false); 
  n2kWind->SetForwardOwnMessages(true);
  n2kWind->SetMsgHandler(HandleNMEA2000MsgWind);
  Serial.println("opening n2kWind");
  n2kWind->Open();
#else
  // Initialize the CAN port
    n2kWind.reset();
    int cRetCode;
    if ((n2kWind.setBitrate(CAN_250KBPS, MCP_16MHZ) == CAN_OK) && (cRetCode = n2kWind.setListenOnlyMode() == CAN_OK))
        Serial.println("CAN0 Initialized.");
    else {
        Serial.println("CAN0 Initialization Error.");
    }
#endif
 
  if (!SPIFFS.begin())
    Serial.println("An error has occurred while mounting SPIFFS");
  else
    Serial.println("SPIFFS mounted successfully");

  Serial.print("ESP local MAC addr: ");
  Serial.println(WiFi.macAddress());

#if defined(CMPS14)
  // check compass
  Wire.begin(SDA_PIN,SCL_PIN);
  Wire.beginTransmission(CMPS14_ADDRESS);
  // null return indicates no error; compass present
  if (compassReady = (!Wire.endTransmission()))
    Serial.println("CMPS14 present");
  else {
    Serial.println("CMPS14 not found");
  }
#endif
#ifdef BNO08X
  compassReady = bno08x.begin_I2C(ADABNO);
  if (compassReady) {
    Serial.println("BNO08x Found");
    for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
      String logString = "Part " + String(bno08x.prodIds.entry[n].swPartNumber) + ": Version :" + String(bno08x.prodIds.entry[n].swVersionMajor) + "." + String(bno08x.prodIds.entry[n].swVersionMinor) + "." + String(bno08x.prodIds.entry[n].swVersionPatch) + " Build " + String(bno08x.prodIds.entry[n].swBuildNumber);
      Serial.println(logString);
    }
    if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 100))
      Serial.println("Could not enable rotation vector");
    else
      Serial.println("enabled abs rotation vector");
  } else
    Serial.printf("Failed to find BNO08x chip @ 0x%x\n", ADABNO);
#endif

  setupWifi();
  startWebServer();
  serverStarted=true;

  ElegantOTA.begin(&server);

  WebSerial.begin(&server);
  WebSerial.onMessage(WebSerialonMessage);

  Serial.printf("flash size 0x%x\n", ESP.getFlashChipSize()); // 4194304

  // No need to parse the messages at every single loop iteration; 1 ms will do
  app.onRepeat(10, []() {
    PollCANStatus();
    n2kMain->ParseMessages();
#if defined(ESPBERRY) || defined(PICANM)
    ParseWindCAN();
#endif
#ifdef PICANMX
    n2kWind->ParseMessages();
#endif
#ifdef PICANM
    NMEA0183_3.ParseMessages(); // GPS from ICOM
#endif
  });
/*
  app.onRepeat(100, []() {
    if (time_since_last_can_rx > MAX_RX_WAIT_TIME_MS) {
      // No CAN messages received in a while; reboot
      esp_task_wdt_init(1, true);
      esp_task_wdt_add(NULL);
      while (true) {
        // wait for watchdog to trigger
      }
    }
  });
*/

  // update web page
  app.onRepeat(WebTimerDelay, []() {
    //Serial.println("transmit sensor readings");
    // Send Events to the client with the Sensor Readings Every x seconds
    events.send("ping",NULL,millis());
    events.send(getSensorReadings().c_str(),"new_readings" ,millis());
  });

  app.onRepeat(100, []() {
    drd->loop(); // double reset detector
    WebSerial.loop();
    ElegantOTA.loop();
  });

// if wifi not connected, we're only going to attempt reconnect once every 5 minutes
// if we get in range, it's simpler to reboot than to constantly check
  app.onRepeat(300000, []() {
    check_status(); // wifi
  });

  // check in for new heading 100 msecs = 10Hz
  if (compassReady)
    app.onRepeat(500, []() {
      //Serial.println("check for new heading");
      // Heading, corrected for local variation (acquired from ICOM via NMEA0183)
      // TBD: set Variation if we get a Heading PGN on main bus that includes it
      // this will be used in settings.html and compass.html
      BoatData.TrueHeading = getCompass(BoatData.Variation);
      // the global boatCompassDeg will always contain the boat compass (magnetic), adjusted for orientation between the boat compass and the mast compass
      boatCompassDeg = getCompass(mastOrientation);
    });
    else Serial.println("compass not ready no heading reaction");

  // update results
  app.onRepeat(1000, []() {
    //Serial.println("update results");
    if (displayOnToggle)
      OLEDdataWindDebug();
    else {
      //display.clearDisplay();
      //display.display();
      // this might be kinda weird but we have to clear counters sometime because otherwise they roll off screen
      num_n2k_messages = 0;
      num_wind_messages = 0;
    }
  });

}

void loop() { app.tick(); }
