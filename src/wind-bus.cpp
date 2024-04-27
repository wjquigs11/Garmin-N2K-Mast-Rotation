/* 
Integrated (single ESP32) mast rotation correction 
Relies on ESPberry to get RPI header for MCP2515-based HAT, since MCP2515 module causes kernel panic
Added PGN for "rudder" to output mast rotation angle from Honeywell sensor
Also reads NMEA0183 data from ICOM VHF and injects it onto N2K bus
Web server for displaying wind speed/angle/mast rotation
TBD: add wiring from TX on Serial(1) to Autohelm plug; port autopilot code here and modify web interface
TBD: translate apparent wind to Seatalk1 and send to tiller pilot
*/

#include <Arduino.h>

#define ESPBERRY
#ifdef ESPBERRY
#define CAN_TX_PIN GPIO_NUM_26 // 26 = IO26, not GPIO26, header pin 16
#define CAN_RX_PIN GPIO_NUM_35
#define N2k_SPI_CS_PIN 5    // If you use mcp_can and CS pin is not 53, uncomment this and modify definition to match your CS pin.
#define N2k_CAN_INT_PIN 22   // If you use mcp_can and interrupt pin is not 21, uncomment this and modify definition to match your interrupt pin.
#endif
#ifdef SH_ESP32
//#define CAN_TX_PIN GPIO_NUM_32
//#define CAN_RX_PIN GPIO_NUM_34
// external transceiver because on-board one isn't working
#define CAN_RX_PIN GPIO_NUM_27
#define CAN_TX_PIN GPIO_NUM_25
#define N2k_SPI_CS_PIN 5    
//#define N2k_CAN_INT_PIN 22   
#define N2k_CAN_INT_PIN 0xFF   
#define MOSI 26
#define MISO 18
#define SS 5
#define SCK 19
#endif

#define SDA_PIN 16
#define SCL_PIN 17
//#define TXD0 18
//#define RXD0 19

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
#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <Arduino.h>
#include <NMEA2000_mcp.h>
//#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
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

TwoWire *i2c;

HardwareSerial NMEA0183serial(1);
#define NMEA0183RX 15 // ESPberry RX0 = IO15 (or is it 16? docs unclear)
extern tBoatData *pBD;
tBoatData BoatData;
tNMEA0183 NMEA0183_3;

//Stream *read_stream = &Serial;
Stream *forward_stream = &Serial;

tNMEA2000 *n2kMain;
tNMEA2000 *n2kWind;

// Honeywell sensor
extern movingAvg honeywellSensor;                // define the moving average object
extern int mastRotate, rotateout;
extern int PotValue, PotLo, PotHi;
extern Adafruit_ADS1015 ads;

int num_n2k_messages = 0;
int num_wind_messages = 0;
elapsedMillis time_since_last_can_rx = 0;

// defs for wifi
extern bool APmodeSwitch;
bool startWifi();
void initSPIFFS();
void initWebSocket();
void APmode();
void notifyClients(String);
extern AsyncWebServer server;
extern char *hostname;
extern int WebTimerDelay;
extern AsyncWebSocket ws;
extern JSONVar readings;

// mast compass
int MagHeading(const tN2kMsg &N2kMsg);
void httpInit(const char* serverName);
extern const char* serverName;
extern int magOrientation;
void mastHeading();
int mastAngle[2]; // array for both sensors

// Time after which we should reboot if we haven't received any CAN messages
#define MAX_RX_WAIT_TIME_MS 30000

void ToggleLed() {
  static bool led_state = false;
  digitalWrite(LED_BUILTIN, led_state);
  led_state = !led_state;
}

void WindSpeed(const tN2kMsg &N2kMsg);

const unsigned long TransmitMessages[] PROGMEM={130306L,0};

/* handle 130306 on the wind bus
tNMEA2000Handler NMEA2000Handlers[]={
  {130306L,&WindSpeed},
  {0,0}
};
*/

// NMEA 2000 message handler for main bus: just count messages
void HandleNMEA2000MsgMain(const tN2kMsg &N2kMsg) {   
  //N2kMsg.Print(&Serial);
  num_n2k_messages++;
  time_since_last_can_rx = 0;
  ToggleLed();
  switch (N2kMsg.PGN) {
    case 127250L:
      // got bearing (from RPI); calculate mast heading
      tN2kMsg correctN2kMsg;
      unsigned char instance=1;
      int mastRotate = MagHeading(N2kMsg);
      if (mastRotate > -1) {
        mastAngle[instance] = (mastRotate * 4068) / 71;   // cheap rads to degree
        // for now (until you dive into SensESP), send rotation angle as rudder (#1)
        SetN2kPGN127245(correctN2kMsg, mastRotate*(M_PI/180), instance, N2kRDO_NoDirectionOrder, 0);
        n2kMain->SendMsg(correctN2kMsg);
        // send on USB as well in case CAN messages can't be differentiated
        correctN2kMsg.SendInActisenseFormat(forward_stream);
      }
    /*
    case 127245L:
      // "rudder" (mast) angle
      double rudPos, angleOrder;
      unsigned char Instance;
      tN2kRudderDirectionOrder Direction;
      ParseN2kPGN127245(N2kMsg, rudPos, Instance, Direction, angleOrder);
      mastAngle[Instance] = (rudPos * 4068) / 71;   // cheap rads to degree
      break;
    */
  } 
}

// NMEA 2000 message handler for wind bus: check if message is wind and handle it
void HandleNMEA2000MsgWind(const tN2kMsg &N2kMsg) {   
  //N2kMsg.Print(&Serial);
  num_wind_messages++;
  time_since_last_can_rx = 0;
  ToggleLed();
  switch (N2kMsg.PGN) {
    case 130306L:
      WindSpeed(N2kMsg);
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
  int windAngleDegrees = WindSensor::windAngleDegrees;
  display->clearDisplay();
  display->setCursor(0, 0);
  display->printf("CAN:%s ", can_state.c_str());
  display->printf("Up:%lu\n", millis() / 1000);
  display->printf("N2K:%d ", num_n2k_messages);
  display->printf("Wind:%d\n", num_wind_messages);
  display->printf("S/A/R:%2.2lf/%2d/%2d\n", windSpeedKnots, windAngleDegrees,rotateout);
  //display->printf("Rot:%d\n", mastRotate);
  display->printf("Sens:%d/%d/%d\n", PotLo, PotValue, PotHi);
  display->printf("Hon: %d, Mag: %d\n", mastAngle[0], mastAngle[1]);
  display->display();
}

void setup() {
  // setup serial output
  Serial.begin(115200);
  delay(100);
  Serial.println("starting wind");

  //adc1_config_width(ADC_WIDTH_BIT_12);
  //adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_MAX);
  
  pinMode(OLED_RESET, OUTPUT);  // RES Pin Display
  digitalWrite(OLED_RESET, LOW);
  delay(500);
  digitalWrite(OLED_RESET, HIGH);

  //display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, i2c, -1);
  display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

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

  honeywellSensor.begin();    // Instantiates the moving average object
  ads.begin();  // start ADS1015 ADC

  // Set up NMEA0183 ports and handlers
  pBD=&BoatData;
  NMEA0183_3.SetMsgHandler(HandleNMEA0183Msg);
  //DebugNMEA0183Handlers(&Serial);
  NMEA0183serial.begin(4800, SERIAL_8N1, NMEA0183RX, -1);
  NMEA0183_3.SetMessageStream(&NMEA0183serial);
  NMEA0183_3.Open();
  if (!NMEA0183serial) 
    Serial.println("failed to open NMEA0183 serial port");

  // instantiate the NMEA2000 object for the main bus
  // switching buses for testing
  //n2kMain = new tNMEA2000_esp32(CAN_TX_PIN, CAN_RX_PIN);
  n2kMain = new tNMEA2000_mcp(N2k_SPI_CS_PIN,MCP_16MHz);
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
  n2kMain->SetForwardStream(forward_stream);
  n2kMain->SetMode(tNMEA2000::N2km_ListenAndSend);
  n2kMain->SetForwardType(tNMEA2000::fwdt_Text); // Show bus data in clear
  n2kMain->EnableForward(true);
  n2kMain->SetForwardOwnMessages(true); // shouldn't have any self-generated messages on main bus
  n2kMain->SetMsgHandler(HandleNMEA2000MsgMain); 
  // I could use the same handler on both since I *should* never get a wind PGN on the main bus
  n2kMain->EnableForward(false);
  n2kMain->SetForwardOwnMessages(true);
  n2kMain->ExtendTransmitMessages(TransmitMessages);
  Serial.println("opening n2kMain");
  n2kMain->Open();

  // instantiate the NMEA2000 object for the wind bus
  // does not need to register since it's only listening
  //n2kWind = new tNMEA2000_mcp(N2k_SPI_CS_PIN,MCP_16MHz);
  n2kWind = new tNMEA2000_esp32(CAN_TX_PIN, CAN_RX_PIN);
  n2kWind->SetN2kCANMsgBufSize(8);
  n2kWind->SetN2kCANReceiveFrameBufSize(100);
  n2kWind->SetMode(tNMEA2000::N2km_ListenAndSend);
  n2kWind->SetForwardStream(forward_stream);
  n2kWind->SetForwardType(tNMEA2000::fwdt_Text);
  n2kWind->EnableForward(false); 
  n2kWind->SetForwardOwnMessages(true);
  n2kWind->SetMsgHandler(HandleNMEA2000MsgWind);
  Serial.println("opening n2kWind");
  n2kWind->Open();

  initSPIFFS();

  if (APmodeSwitch = startWifi()) {
    Serial.print("Init WIFI OK ");
    Serial.println(APmodeSwitch);
    // Initialize mDNS
    if (!MDNS.begin(hostname)) {   
      Serial.println("Error setting up MDNS responder!");
      while(1) delay(1000);
    } else Serial.println("MDNS OK");
    // Web Server Root URL
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      Serial.println("GET /index");
      request->send(SPIFFS, "/index.html", "text/html");
    });
    initWebSocket();
    server.serveStatic("/", SPIFFS, "/");
    // Start server
    server.begin();
  } else // start wifi returned false; set up AP to config wifi
    APmode();

  // No need to parse the messages at every single loop iteration; 1 ms will do
  app.onRepeat(1, []() {
    PollCANStatus();
    n2kMain->ParseMessages();
    n2kWind->ParseMessages();
    NMEA0183_3.ParseMessages(); // GPS from ICOM
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
    if (APmodeSwitch) {
      String jsonString = JSON.stringify(readings);
    //  Serial.println("sending readings from WebTimerDelay");
    //  Serial.println(jsonString);
      notifyClients(jsonString);
    }
    // Not sure if this should go here
    ws.cleanupClients();
  });

  // check in with mast for new heading @ 10Hz
  app.onRepeat(100, []() {
    mastHeading();
  });

  // update results
  app.onRepeat(1000, []() {
    OLEDdataWindDebug();
    //num_n2k_messages = 0;
    //num_wind_messages = 0;
  });
}

void loop() { app.tick(); }
