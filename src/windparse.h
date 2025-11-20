#include <ActisenseReader.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <N2kMessages.h>
#include <NMEA2000_esp32.h>
#include <ReactESP.h>
#include <Wire.h>
#include <movingAvg.h>
#include "elapsedMillis.h"
#include <Adafruit_ADS1X15.h>
#include <SPI.h>
//#include <HTTPClient.h>
#include "esp_system.h"
#include "esp32-hal-log.h"
#include <Adafruit_ADS1X15.h>
#include <Adafruit_Sensor.h>

#include <vector>
#include <numeric>

#define POT_PIN 33 // ESPberry ADC1_CH5
// voltage sensor 5:1 so 12v measures at 2.4 volts so use ADC_ATTEN_DB_12
extern tNMEA2000 *n2kWind;

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
    static int windAngleDegrees;
    static int windAngleRadians;
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

void setupWind();
void loopWind();
int readAnalogRotationValue();
void WindSpeed(const tN2kMsg &N2kMsg);
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);
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

extern int rotateout;

//#define MAXPGN 64
//#define MAX_NETS 5

extern bool displayOnToggle, honeywellOnToggle;
extern bool stackTrace;

#define DEGTORAD 0.01745329252
#define RADTODEG 57.2957795131

extern bool gpsDebug;
extern bool tuning; // for tuning instruments e.g. paddlewheel vs GPS




