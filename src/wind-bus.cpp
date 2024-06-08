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
#include "windparse.h"
//#include <mcp_can.h>
//#include <mcp_can_dfs.h>
#include <Arduino.h>
#include <N2kMessages.h>
//#include <driver/adc.h>
#include <Adafruit_ADS1X15.h>
//#include <Fonts/FreeMono9pt7b.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
#include <Arduino_JSON.h>
#include <ESPmDNS.h>
#include "mcp2515.h"
#include "can.h"

#define ESPBERRY
//#define SH_ESP32  // these defs will probably change with SINGLECAN
//#define SINGLECAN  // for testing (or non-Garmin) we can use one bus 

#ifdef ESPBERRY
#define CAN_TX_PIN GPIO_NUM_26 // 26 = IO26, not GPIO26, RPI header pin 16
#define CAN_RX_PIN GPIO_NUM_35 // RPI header pin 15
//#define N2k_SPI_CS_PIN 5    
//#define N2k_CAN_INT_PIN 26
// espBerry Settings (will need modification for other ESP32 systems)
// ------------------------------------------------------------------------
// Define CAN Ports and their Chip Select/Enable
// the Artist formerly known as "n2kWind"...
//#define CAN_RX_INTERRUPT_MODE 0
MCP2515 n2kWind(5); // CAN0 interface CS
//MCP2515 CAN1(17); // CAN1 interface CS
// Interrupt Signals
const int CAN0_INT = 26;    // RPi Pin 16 - GPIO23 - IO26
//const int CAN1_INT = 34;    // RPi Pin 22 - GPIO25 - IO34
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
#endif

#define SDA_PIN 16
#define SCL_PIN 17
//#define TXD0 18
//#define RXD0 19

using namespace reactesp;

ReactESP app;

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
// RPI pin 24 = CP0/GPIO8 = IO5
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SSD1306 *display;

#define RECOVERY_RETRY_MS 1000  // How long to attempt CAN bus recovery

TwoWire *i2c, *i2c2;

//#define PARSENMEA0183
#ifdef PARSENMEA0183
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

// defs for robotshop CMPS14
int CMPS14_ADDRESS=0x60;
bool cmps14_ready=false;

//Stream *read_stream = &Serial;
Stream *forward_stream = &Serial;

tNMEA2000 *n2kMain;

// Honeywell sensor
extern movingAvg honeywellSensor;                // define the moving average object
extern int mastRotate, rotateout;
extern int PotValue, PotLo, PotHi;
extern Adafruit_ADS1015 ads;
extern int adsInit;

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
extern char *hostname;
extern int WebTimerDelay;
//extern AsyncWebSocket ws;
extern AsyncEventSource events;
extern JSONVar readings;
extern void setupWifi();
extern String host;
extern void loopWifi();
void startWebServer();
String getSensorReadings();
void setupESPNOW();
void sendMastControl();

// mast compass
int convertMagHeading(const tN2kMsg &N2kMsg); // magnetic heading of boat (from e.g. B&G compass)
float parseMastHeading(const tN2kMsg &N2kMsg);  // mast heading
float getCompass(int correction);      // boat heading from internal ESP32 CMPS14
void httpInit(const char* serverName);
extern const char* serverName;
extern int mastOrientation;   // delta between mast compass and boat compass
extern float boatCompassDeg; // magnetic heading not corrected for variation
extern float mastCompassDeg;
void mastHeading();
int mastAngle[2]; // array for both sensors
// 0 = honeywell
// 1 = compass

// Time after which we should reboot if we haven't received any CAN messages
#define MAX_RX_WAIT_TIME_MS 30000

bool displayOnToggle=true, compassOnToggle=false, honeywellOnToggle=false;

void ToggleLed() {
  static bool led_state = false;
  digitalWrite(LED_BUILTIN, led_state);
  led_state = !led_state;
}

//void WindSpeed(const tN2kMsg &N2kMsg);
void WindSpeed();
void BoatSpeed(const tN2kMsg &N2kMsg);

const unsigned long TransmitMessages[] PROGMEM={130306L,0};
// do I need to add mag heading?

/* if you add more handlers you might want to switch back to this method instead of switch/case
tNMEA2000Handler NMEA2000Handlers[]={
  {130306L,&WindSpeed},
  {0,0}
};
*/

// NMEA 2000 message handler for main bus
void HandleNMEA2000MsgMain(const tN2kMsg &N2kMsg) {   
  Serial.print("main: "); N2kMsg.Print(&Serial);
  num_n2k_messages++;
  time_since_last_can_rx = 0;
  ToggleLed();
  switch (N2kMsg.PGN) {
#ifdef OTHER_COMPASS  // we only care about 127250 if there is already a boat compass on the N2K bus
    case 127250L: // magnetic heading
#ifdef DEBUG
      Serial.println(N2kMsg.PGN);
#endif
      if (compassOnToggle) {
        // got bearing; calculate mast heading
        tN2kMsg correctN2kMsg;
        int mastRotate = convertMagHeading(N2kMsg);
        if (mastRotate > -1) {
          // for now (until you dive into SensESP), send rotation angle as "rate of turn"
          SetN2kPGN127251(correctN2kMsg,0xff,(mastRotate+50)*(M_PI/180));
          n2kMain->SendMsg(correctN2kMsg);
        }
      }
      // set BoatData here?
      break;
#endif // OTHER_COMPASS
#ifdef SINGLECAN
    case 130306L:
      WindSpeed(N2kMsg);
      break;
    case 127250L: // heading; on wind bus should only come from mast compass
      int mastComp;
      if ((mastComp = parseMastHeading(N2kMsg)) > -1) {
        mastCompassDeg = mastComp;
        boatCompassDeg = getCompass(mastOrientation);
      }
      //Serial.printf("PGN Mast Heading: %.2f Boat Heading: %.2f\n", mastCompassDeg, boatCompassDeg);
      break;
    case 128259L:
      BoatSpeed(N2kMsg);
      break;
#endif
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

#if 0 // obsolete since we're parsing natively
// NMEA 2000 message handler for wind bus: check if message is wind and handle it
// also checking for Heading (from mast compass)
void HandleNMEA2000MsgWind(const tN2kMsg &N2kMsg) {   
  Serial.print("wind: "); N2kMsg.Print(&Serial);
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
    case 127250L: // heading; on wind bus should only come from mast compass
      float mastComp;
      if ((mastComp = parseMastHeading(N2kMsg)) > -1) {
        mastCompassDeg = mastComp; // incorrect? radians
        // we get boatCompassDeg here but we should also do it on schedule so the ship's compass is still valid even if we're not connected to mast compass
        boatCompassDeg = getCompass(mastOrientation);
      }
      Serial.printf("PGN Mast Heading: %.2f Boat Heading: %.2f\n", mastCompassDeg, boatCompassDeg);
      break;
    case 128259L:
      BoatSpeed(N2kMsg);
      break;
  } 
  /* this will be useful if we're handling more than one message type later
  int iHandler;
  for (iHandler=0; NMEA2000Handlers[iHandler].PGN!=0 && !(N2kMsg.PGN==NMEA2000Handlers[iHandler].PGN); iHandler++);
  if (NMEA2000Handlers[iHandler].PGN!=0) {
    NMEA2000Handlers[iHandler].Handler(N2kMsg); 
  }
  */
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
  display->clearDisplay();
  display->setCursor(0, 5);
  display->printf("CAN: %s ", can_state.c_str());
  display->printf("Up: %lu\n", millis() / 1000);
  display->printf("N2K: %d ", num_n2k_messages);
  display->printf("Wind: %d\n", num_wind_messages);
  display->printf("S/A/R:%2.1f/%2.1f/%2.1f\n", windSpeedKnots, windAngleDegrees,rotateout);
  //display->printf("Rot:%d\n", mastRotate);
  display->printf("Sensor: %d/%d/%d\n", PotLo, PotValue, PotHi);
  display->printf("Angle: %d\n", mastAngle[0]);
  display->printf("M: %.1f B: %.1f\n", mastCompassDeg, boatCompassDeg);
  display->printf("Delta: %d\n", mastAngle[1]);
  //Serial.printf("Hon: %d, Mag: %d\n", mastAngle[0], mastAngle[1]);
  display->display();
}

// used for CAN TBD clean up
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];                        // Array to store serial string
void ParseWindCAN();
byte canInit(const int canPort, const CAN_SPEED canSpeed, const int canOpMode);

void setup() {
  // setup serial output
  Serial.begin(115200);
  delay(100);
  Serial.println("starting wind correction");

  //adc1_config_width(ADC_WIDTH_BIT_12);
  //adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_MAX);
  
#ifdef ESPBERRY
  pinMode(OLED_RESET, OUTPUT);  // RES Pin Display
  digitalWrite(OLED_RESET, LOW);
  delay(500);
  digitalWrite(OLED_RESET, HIGH);
  display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#endif
#ifdef SH_ESP32
  i2c = new TwoWire(0);
  i2c->begin(SDA_PIN, SCL_PIN);
  display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, i2c, -1);
#endif
  if(!display->begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
    Serial.println(F("SSD1306 allocation failed"));
  else
    Serial.println(F("SSD1306 allocation success"));
  display->setTextSize(1);
  display->setTextColor(WHITE);
  display->setRotation(2);
  //display->setFont(&FreeMono9pt7b);  
  display->setFont(NULL);  

  // toggle the LED pin at rate of 1 Hz
  pinMode(LED_BUILTIN, OUTPUT);
  app.onRepeatMicros(1e6 / 1, []() { ToggleLed(); });

  if (!(adsInit = ads.begin())) {  // start ADS1015 ADC
    Serial.println("ads sensor not found");
    honeywellOnToggle = false;
  } else honeywellSensor.begin();    // Instantiates the moving average object

  // Set up NMEA0183 ports and handlers
  pBD=&BoatData;
#ifdef PARSENMEA0183
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
      "20210331",  // Manufacturer's Model serial code (max 32 chars)
      103,         // Manufacturer's product code
      "SH-ESP32 Wind Correction",  // Manufacturer's Model ID (max 33 chars)
      "0.1.0.0 (2021-03-31)",  // Manufacturer's Software version code (max 40
                               // chars)
      "0.0.3.1 (2021-03-07)"   // Manufacturer's Model version (max 24 chars)
  );
  // Set device information
  n2kMain->SetDeviceInformation(
      1,    // Unique number. Use e.g. Serial number.
      130,  // Device function=Analog to NMEA 2000 Gateway. See codes on
            // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
      25,   // Device class=Inter/Intranetwork Device. See codes on
           // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
      2046  // Just choosen free from code list on
            // http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
  );
  //n2kMain->SetForwardStream(forward_stream);
  n2kMain->SetMode(tNMEA2000::N2km_ListenAndSend);
  n2kMain->SetForwardType(tNMEA2000::fwdt_Text); // Show bus data in clear
  //n2kMain->EnableForward(true);
  //n2kMain->EnableForward(false);
  //n2kMain->SetForwardOwnMessages(true);
  n2kMain->SetMsgHandler(HandleNMEA2000MsgMain); 
  n2kMain->ExtendTransmitMessages(TransmitMessages);
  Serial.println("opening n2kMain");
  n2kMain->Open();

#ifndef SINGLECAN
  // Initialize the CAN port
    n2kWind.reset();
    int cRetCode;
    if ((n2kWind.setBitrate(CAN_250KBPS, MCP_16MHZ) == CAN_OK) && (cRetCode = n2kWind.setListenOnlyMode() == CAN_OK))
        Serial.println("CAN0 Initialized.");
    else {
        Serial.println("CAN0 Initialization Error.");
        return;
    }

  /*
  if(canInit(0, CAN_250KBPS, CAN_OP_NORMAL) == CAN_OK)
        Serial.println("CAN0 Initialized.");
    else
  //n2kWind.setMode(CAN_OP_LISTENONLY);
  //pinMode(CAN0_INT, INPUT);
  */
#endif
 
  if (!SPIFFS.begin())
    Serial.println("An error has occurred while mounting SPIFFS");
  else
    Serial.println("SPIFFS mounted successfully");

  Serial.print("ESP local MAC addr: ");
  Serial.println(WiFi.macAddress());

  // check compass
  //Wire1.setPins(21,22); // SDA (purple), SCL (orange)
  Wire.begin(21,22);
  Wire.beginTransmission(CMPS14_ADDRESS);
  cmps14_ready = (!Wire.endTransmission()); // null return indicates no error; compass present
  if (cmps14_ready)
    Serial.println("CMPS14 present");
  else {
    Serial.println("CMPS14 not found");
    return;
  }

  setupWifi();
  startWebServer();

  setupESPNOW();
  // send settings to mast compass ESP on startup
  sendMastControl();

  Serial.printf("flash size 0x%x\n", ESP.getFlashChipSize()); // 4194304

  // No need to parse the messages at every single loop iteration; 1 ms will do
  app.onRepeat(10, []() {
    PollCANStatus();
    n2kMain->ParseMessages();
#ifndef SINGLECAN
    ParseWindCAN();
#endif
#ifdef PARSENMEA0183
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

  // check for DRD
  app.onRepeat(1000, []() {
    //Serial.println("check for DRD");
    loopWifi();
  });

  // check in for new heading 100 msecs = 10Hz
  if (cmps14_ready)
    app.onRepeat(300, []() {
      //Serial.println("check for new heading");
      // Heading, corrected for local variation (acquired from ICOM via NMEA0183)
      // TBD: set Variation if we get a Heading PGN on main bus that includes it
      // this will be used in settings.html and compass.html
      BoatData.TrueHeading = getCompass(BoatData.Variation);
      // the global boatCompassDeg will always contain the boat compass (magnetic), adjusted for orientation between the boat compass and the mast compass
      boatCompassDeg = getCompass(mastOrientation);
    });

  // update results
  app.onRepeat(1000, []() {
    //Serial.println("update results");
    if (displayOnToggle)
      OLEDdataWindDebug();
    else {
      display->clearDisplay();
      display->display();
      // this might be kinda weird but we have to clear counters sometime because otherwise they roll off screen
      num_n2k_messages = 0;
      num_wind_messages = 0;
    }
    //Serial.printf("Boat Heading: %.2f Variation %.2f\n", BoatData.TrueHeading, BoatData.Variation);
  });

}

void loop() { app.tick(); }
