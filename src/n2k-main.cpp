#if defined(N2K) && defined(MAINBUS)
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
#define RX 12
#define TX 14

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