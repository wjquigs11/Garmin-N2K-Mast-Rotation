#include <Arduino.h>
#include <ActisenseReader.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <N2kMessages.h>
#include <NMEA2000_esp32.h>
#ifdef NMEA0183
#include <NMEA0183Msg.h>
#include <NMEA0183Messages.h>
#include "NMEA0183Handlers.h"
#endif
#include <ReactESP.h>
#include <Wire.h>
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>
#include <movingAvg.h>
#include "elapsedMillis.h"
#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include <N2kMessages.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <SPIFFS.h>
#include <ESPmDNS.h>
#include <SPI.h>
#include <ESPAsyncWebServer.h>
//#include <WebSerial.h>
#include <WebSerialPro.h>
#include <ElegantOTA.h>
#include <HTTPClient.h>
#include "esp_system.h"
#include "esp32-hal-log.h"
#include <Arduino_JSON.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <NTPClient.h>
#ifdef PICAN
#include <Adafruit_ADS1X15.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#endif

#define SPI_CS_PIN 5
#define CAN_INT_PIN 21
#define MAX_DATA_SIZE 8

#ifdef SH_ESP32
#define POT_PIN 36 // SH_ESP32
#endif
#if defined(PICAN)
#define POT_PIN 33 // ESPberry ADC1_CH5
// voltage sensor 5:1 so 12v measures at 2.4 volts so use ADC_ATTEN_DB_12
#endif

class RotationSensor {
  public:
    static int newValue;
    static int oldValue;
};

class WindSensor {
  public:
    static double windSpeedKnots;
    static double windSpeedMeters;
    static double windAngleDegrees;
    static double windAngleRadians;
};

typedef struct {
  unsigned long PGN;
  void (*Handler)(const tN2kMsg &N2kMsg); 
} tNMEA2000Handler;

void ParseWindN2K(const tN2kMsg &N2kMsg);
void parseWindCAN();
void WindSpeed();

extern tNMEA2000Handler NMEA2000Handlers[];

float readAnalogRotationValue();
void parseWindCAN();
double ReadWindAngle(int);
double ReadWindSpeed();
int readWindAngleInput();
void SendN2kWind(int);

#define WindUpdatePeriod 500

// timing/display
extern int num_n2k_messages;
extern int num_wind_messages;
extern elapsedMillis time_since_last_can_rx;
extern elapsedMillis time_since_last_wind_rx;
extern unsigned long total_time_since_last_wind;
extern unsigned long avg_time_since_last_wind;
extern String host;

extern bool imuReady;

#ifdef DISPLAYON
extern Adafruit_SSD1306 *display;
#endif

#define PRBUF 256
extern char prbuf[PRBUF];

// Honeywell observed range
#define lowset 56
#define highset 311
extern int mastAngle[];

#define MAXPGN 64
#define MAX_NETS 4

#define SEALEVELPRESSURE_HPA (1013.25)
extern Adafruit_BME280 bme;
extern bool bmeFound;

#define MAXSAT 140 // no idea how many satellites there are 
extern struct tGSV GSVseen[]; 
extern int maxSat;
extern bool GSVtoggle;

extern bool displayOnToggle, honeywellOnToggle;
extern bool stackTrace;

#define DEGTORAD 0.01745329252
#define RADTODEG 57.2957795131