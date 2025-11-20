#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ReactESP.h>
#include <Wire.h>
//#include <esp_int_wdt.h>
//#include <esp_task_wdt.h>
#include <movingAvg.h>
#include "elapsedMillis.h"
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
#include <Adafruit_ADS1X15.h>
#include <Adafruit_Sensor.h>

#ifdef NTP
#include <NTPClient.h>
#endif

//#define POT_PIN 33 // ESPberry ADC1_CH5
// voltage sensor 5:1 so 12v measures at 2.4 volts so use ADC_ATTEN_DB_12
extern tNMEA2000 *n2kWind;
#endif
extern bool logPot;
extern bool passThrough;
extern bool debugNMEA;
extern bool debugN2K;

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
extern tNMEA2000 *n2kMain;

extern bool windForward;
extern bool mainForward;

float readAnalogRotationValue();
void parseWindCAN();
double ReadWindAngle(int);
double ReadWindSpeed();
int readWindAngleInput();
void SendN2kWind(int);

void calcTrueWindDirection(); // absolute
void calcTrueWindAngle(); // relative to bow

#define WindUpdatePeriod 500

extern Preferences preferences;

// timing/display
extern int num_n2k_messages;
extern int num_wind_messages;
extern elapsedMillis time_since_last_can_rx;
extern elapsedMillis time_since_last_wind_rx;
extern unsigned long total_time_since_last_wind;
extern unsigned long avg_time_since_last_wind;
extern String host;

#ifdef DISPLAYON
extern Adafruit_SSD1306 *display;
#endif

#define PRBUF 256
extern char prbuf[PRBUF];

// Honeywell observed range
// TBD: make this dynamic based on min/max over a long runtime
//#define lowset 56
//#define highset 311
#define lowset 11
#define highset 244
extern int mastAngle[];

extern float mastOrientation; // mast compass position relative to boat compass position
extern int sensOrientation; // Honeywell orientation relative to centerline
extern int boatOrientation; // boat compass position relative to centerline
extern int rtkOrientation;
extern int mastAngle[];
extern float mastRotate, rotateout;
extern uint8_t compassAddress[];
extern float mastCompassDeg;
extern int boatCalStatus;

#define MAXPGN 64
#define MAX_NETS 5

extern bool displayOnToggle, honeywellOnToggle;
extern bool stackTrace;

#define DEGTORAD 0.01745329252
#define RADTODEG 57.2957795131



