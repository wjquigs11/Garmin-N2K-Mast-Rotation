#define N2K
#define MAINBUS
#if defined(N2K) && defined(MAINBUS)
#include <Arduino.h>
#include <N2kMsg.h>
#include <NMEA2000.h>
#include <NMEA2000_CAN.h>
#include <ActisenseReader.h>
#include <N2kMessages.h>
#include <NMEA2000_esp32.h>
#include <ReactESP.h>

#include "compass.h"
#include "windparse.h"
#include "BoatData.h"

// object-oriented classes
#include "BNO085Compass.h"
#include "n2k.h"
#include "logto.h"

#include <Arduino.h>

#define CAN_RX_PIN GPIO_NUM_34
#define CAN_TX_PIN GPIO_NUM_32

#include "elapsedMillis.h"

extern tBoatData *pBD;
tBoatData BoatData;
// TBD: I should add mag heading, mast heading etc (all the globals) to BoatData struct so I could have a single extern in each file

HardwareSerial SerialPort(2);
#define RX 15
#define TX 13

// reading Actisense from wind ESP32, sending main N2K on serial
Stream *read_stream = &SerialPort;
Stream *forward_stream = &SerialPort;

tActisenseReader actisense_reader;

tNMEA2000 *nmea2000;

int num_n2k_messages = 0;
int num_actisense_messages = 0;
elapsedMillis time_since_last_can_rx = 0;

using namespace reactesp;

ReactESP app;

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;
bool bmeFound;

#ifdef HONEY
// Honeywell sensor
extern movingAvg honeywellSensor;                // define the moving average object
extern float mastRotate;
extern int PotValue, PotLo, PotHi;
extern Adafruit_ADS1015 ads;
extern int adsInit;
#endif
extern float rotateout;

// mast compass
int convertMagHeading(const tN2kMsg &N2kMsg); // magnetic heading of boat (from e.g. B&G compass)
float parseMastHeading(const tN2kMsg &N2kMsg);  // mast heading
float parseN2KHeading(const tN2kMsg &N2kMsg); // boat heading from external source
//float getCompass(int correction);      // boat heading from internal ESP32 compass
//void httpInit(const char* serverName);
//extern const char* serverName;
int mastOrientation; // delta between mast compass and boat compass
int sensOrientation; // delta between mast centered and Honeywell sensor reading at center
int boatOrientation; // delta between boat compass and magnetic north
float boatCompassTrue;
float mastCompassDeg;
float mastDelta;
extern tN2kWindReference wRef;
movingAvg mastCompDelta(10);
void mastHeading();
int mastAngle[2]; // array for both sensors
// 0 = honeywell
// 1 = compass
float getMastHeading();

#define W1SCL 33
#define W1SDA 32

bool imuReady=false;
bool mastIMUready=false;

BNO085Compass compass;

int headingErrCount;

bool displayOnToggle=true, honeywellOnToggle=false, demoModeToggle=false;

void HandleStreamN2kMsg(const tN2kMsg &message) {
  num_n2k_messages++;
  time_since_last_can_rx = 0;
  n2k::ToggleLed();
}

void HandleStreamActisenseMsg(const tN2kMsg &message) {
  num_actisense_messages++;
  n2k::ToggleLed();
  //nmea2000->SendMsg(message);
}

void setup() {
  // setup serial output
  Serial.begin(115200); delay(100);
  Serial.println("main bus");

  SerialPort.begin(BAUD, SERIAL_8N1, RX, TX);

  // toggle the LED pin at rate of 1 Hz
  pinMode(LED_BUILTIN, OUTPUT);
  //app.onRepeatMicros(1e6 / 1, []() { ToggleLed(); });

  // instantiate the NMEA2000 object
  nmea2000 = new tNMEA2000_esp32(CAN_TX_PIN, CAN_RX_PIN);

  // input the NMEA 2000 messages

  // Reserve enough buffer for sending all messages. This does not work on small
  // memory devices like Uno or Mega
  nmea2000->SetN2kCANSendFrameBufSize(250);
  nmea2000->SetN2kCANReceiveFrameBufSize(250);

  // Set Product information
  nmea2000->SetProductInformation(
      "20210331",  // Manufacturer's Model serial code (max 32 chars)
      103,         // Manufacturer's product code
      "SH-ESP32 NMEA 2000 USB GW",  // Manufacturer's Model ID (max 33 chars)
      "0.1.0.0 (2021-03-31)",  // Manufacturer's Software version code (max 40
                               // chars)
      "0.0.3.1 (2021-03-07)"   // Manufacturer's Model version (max 24 chars)
  );
  // Set device information
  nmea2000->SetDeviceInformation(
      11111,    // Unique number. Use e.g. Serial number.
      130,  // Device function=Analog to NMEA 2000 Gateway. See codes on
            // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
      25,   // Device class=Inter/Intranetwork Device. See codes on
           // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
      2046  // Just choosen free from code list on
            // http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
  );

  nmea2000->SetForwardStream(forward_stream);
  //nmea2000->EnableForward(true);
  nmea2000->SetMode(tNMEA2000::N2km_ListenAndNode);
  //nmea2000->SetForwardType(tNMEA2000::fwdt_Text);
  //nmea2000->SetForwardOwnMessages(false);
  nmea2000->SetMsgHandler(HandleStreamN2kMsg);
  nmea2000->Open();

  actisense_reader.SetReadStream(read_stream);
  actisense_reader.SetDefaultSource(75);
  actisense_reader.SetMsgHandler(HandleStreamActisenseMsg);

  // No need to parse the messages at every single loop iteration; 1 ms will do
  app.onRepeat(1, []() {
    n2k::PollCANStatus();
    nmea2000->ParseMessages();
    actisense_reader.ParseMessages();
  });

  app.onRepeat(1000, []() {
    Serial.printf("CAN: %s\n", n2k::can_state.c_str());
    Serial.printf("Uptime: %lu\n", millis() / 1000);
    Serial.printf("N2K: %d\n", num_n2k_messages);
    Serial.printf("ACT: %d\n", num_actisense_messages);
    //num_n2k_messages = 0;
    //num_actisense_messages = 0;
  });
}

void loop() { app.tick(); }

#endif

#if defined(N2K) && defined(MAINBUS) && defined(NOTDEF)

tActisenseReader ActisenseReader;

HardwareSerial SerialPort(2);
#define RX 15
#define TX 13

// Define READ_STREAM to port, where you write data from PC e.g. with NMEA Simulator.
#define READ_STREAM SerialPort       
// Define ForwardStream to port, what you listen on PC side. On Arduino Due you can use e.g. SerialUSB
#define FORWARD_STREAM Serial    

Stream *ReadStream=&READ_STREAM;
Stream *ForwardStream=&FORWARD_STREAM;

void HandleStreamN2kMsg(const tN2kMsg &N2kMsg);

void setup() {
  Serial.begin(115200); delay(500);
  Serial.println("NMEA2000 main bus");
  // Define buffers big enough
  NMEA2000.SetN2kCANSendFrameBufSize(150);
  NMEA2000.SetN2kCANReceiveFrameBufSize(150);
  
  if (ReadStream!=ForwardStream) READ_STREAM.begin(115200, SERIAL_8N1, RX, TX);
  FORWARD_STREAM.begin(115200, SERIAL_8N1, RX, TX);
  NMEA2000.SetForwardStream(ForwardStream); 
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndSend);
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show bus data in clear text
  if (ReadStream==ForwardStream) NMEA2000.SetForwardOwnMessages(false); // If streams are same, do not echo own messages.
  // NMEA2000.EnableForward(false);
  NMEA2000.Open();

  // I originally had problem to use same Serial stream for reading and sending.
  // It worked for a while, but then stopped. Later it started to work.
  ActisenseReader.SetReadStream(ReadStream);
  ActisenseReader.SetDefaultSource(75);
  ActisenseReader.SetMsgHandler(HandleStreamN2kMsg); 
}

void HandleStreamN2kMsg(const tN2kMsg &N2kMsg) {
  N2kMsg.Print(&Serial);
  //NMEA2000.SendMsg(N2kMsg,-1);
}

void loop() {
  NMEA2000.ParseMessages();
  ActisenseReader.ParseMessages();
}


#endif

#if defined(N2K) && defined(MAINBUS) && defined(NOTDEF)
#include <Arduino.h>
#include "compass.h"
#include "windparse.h"
#include "BoatData.h"
#include <ActisenseReader.h>

// object-oriented classes
#include "n2k.h"
#include "logto.h"

#define CAN_RX_PIN GPIO_NUM_34
#define CAN_TX_PIN GPIO_NUM_32

#define SDA_PIN 16
#define SCL_PIN 17

#include <ActisenseReader.h>
#include <N2kMessages.h>
#include <NMEA2000_esp32.h>
#include <ReactESP.h>
#include <Wire.h>
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>

#include "elapsedMillis.h"

using namespace reactesp;

ReactESP app;

#define RECOVERY_RETRY_MS 1000  // How long to attempt CAN bus recovery

HardwareSerial SerialPort(2);
#define RX 17
#define TX 16

Stream *read_stream = &SerialPort;
Stream *forward_stream = &SerialPort;

tActisenseReader actisense_reader;

tNMEA2000 *nmea2000;

int num_n2k_messages = 0;
int num_actisense_messages = 0;
elapsedMillis time_since_last_can_rx = 0;

// Time after which we should reboot if we haven't received any CAN messages
#define MAX_RX_WAIT_TIME_MS 30000

void HandleStreamN2kMsg(const tN2kMsg &message) {
  //switch (message.PGN) {
  //  case 127250L:
      Serial.print(time_since_last_can_rx); Serial.print(" ");
      message.Print(&Serial);
  //    break;
  //}
  num_n2k_messages++;
  time_since_last_can_rx = 0;
  n2k::ToggleLed();
}

void HandleStreamActisenseMsg(const tN2kMsg &message) {
  message.Print(&Serial);
  num_actisense_messages++;
  n2k::ToggleLed();
  nmea2000->SendMsg(message);
}

void setup() {
  // setup serial output
  Serial.begin(115200); delay(100);

  SerialPort.begin(BAUD, SERIAL_8N1, RX, TX);

  // toggle the LED pin at rate of 1 Hz
  pinMode(LED_BUILTIN, OUTPUT);
  app.onRepeatMicros(1e6 / 1, []() { n2k::ToggleLed(); });

  // instantiate the NMEA2000 object
  nmea2000 = new tNMEA2000_esp32(CAN_TX_PIN, CAN_RX_PIN);

  // input the NMEA 2000 messages

  // Reserve enough buffer for sending all messages. This does not work on small
  // memory devices like Uno or Mega
  nmea2000->SetN2kCANSendFrameBufSize(250);
  nmea2000->SetN2kCANReceiveFrameBufSize(250);

  // Set Product information
  nmea2000->SetProductInformation(
      "20210331",  // Manufacturer's Model serial code (max 32 chars)
      103,         // Manufacturer's product code
      "SH-ESP32 NMEA 2000 USB GW",  // Manufacturer's Model ID (max 33 chars)
      "0.1.0.0 (2021-03-31)",  // Manufacturer's Software version code (max 40
                               // chars)
      "0.0.3.1 (2021-03-07)"   // Manufacturer's Model version (max 24 chars)
  );
  // Set device information
  nmea2000->SetDeviceInformation(
      1,    // Unique number. Use e.g. Serial number.
      130,  // Device function=Analog to NMEA 2000 Gateway. See codes on
            // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
      25,   // Device class=Inter/Intranetwork Device. See codes on
           // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
      2046  // Just choosen free from code list on
            // http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
  );

  nmea2000->SetForwardStream(forward_stream);
  nmea2000->SetMode(tNMEA2000::N2km_ListenAndNode);
  //nmea2000->SetForwardType(tNMEA2000::fwdt_Text);
  //nmea2000->SetForwardOwnMessages(false);
  nmea2000->SetMsgHandler(HandleStreamN2kMsg);
  nmea2000->Open();

  actisense_reader.SetReadStream(read_stream);
  actisense_reader.SetDefaultSource(75);
  actisense_reader.SetMsgHandler(HandleStreamActisenseMsg);

  // No need to parse the messages at every single loop iteration; 1 ms will do
  app.onRepeat(1, []() {
    //PollCANStatus();
    nmea2000->ParseMessages();
    actisense_reader.ParseMessages();
  });

  app.onRepeat(1000, []() {
    Serial.printf("(%ld) in: %d out: %d: last CAN rx: %ld\n", millis()*1000, num_n2k_messages, num_actisense_messages, time_since_last_can_rx);
  });

#if 0
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
#endif
}

void loop() { app.tick(); }
#endif
