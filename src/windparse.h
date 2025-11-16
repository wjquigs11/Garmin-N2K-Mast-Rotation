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
//#include <esp_int_wdt.h>
//#include <esp_task_wdt.h>
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
#ifdef GPX
#include <FS.h>
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

extern bool imuReady;

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

#ifdef PICAN
#define SEALEVELPRESSURE_HPA (1013.25)
extern Adafruit_BME280 bme;
extern bool bmeFound;
#endif

#define MAXSAT 140 // no idea how many satellites there are 
extern struct tGSV GSVseen[]; 
extern int maxSat;
extern bool GSVtoggle;

extern bool displayOnToggle, honeywellOnToggle;
extern bool stackTrace;

#define DEGTORAD 0.01745329252
#define RADTODEG 57.2957795131

#ifdef RTK
extern bool rtkDebug;
#endif
extern bool gpsDebug;
extern bool tuning; // for tuning instruments e.g. paddlewheel vs GPS

#ifdef GPX
// GPX files
extern char logFile[];
extern char logTempName[];
extern File GPXlogFile, WPTlogFile;
extern char GPXlog[];
extern int lastLogFile;  // Store the last update time
#define NEWLOGFILE 600000 // start a new .gpx file every 10 minutes
extern int logFileIdx;
void writeGPXHeader(File &file);
void writeTrackPoint(File &file, double lat, double lon, double speed, float heading, double timestamp);
void closeGPXFile(File &file);
void writeHeadingWaypoint(File &file, double lat, double lon, double speed, float heading, double timestamp);
void startNextLog();
void initGPX();
#define GPXTIMER 5000
#endif

#ifdef WINDLOG
extern bool windLogging;
extern File windLogFile;
extern int windLogFileIdx;
extern char windLog[];
#define WINDTIMER 1000
void writeWindPoint(File &file, unsigned long timestamp, float awa, double aws, double stw, double twa, double tws, double twd, double vmg, double heading);
void initWindLog();
void startNextWindLog();
#endif

#ifdef INA219
#include <Adafruit_INA219.h>
extern Adafruit_INA219 ina219;
#endif

#ifdef BNO08X000
extern BNO085Compass compass;
#endif

