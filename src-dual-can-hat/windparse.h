#include "elapsedMillis.h"

#define SPI_CS_PIN 5
#define CAN_INT_PIN 21
#define MAX_DATA_SIZE 8

void IRAM_ATTR NewDataReadyISR();

#define ESPBERRY
//#define SH_ESP32  // these defs will probably change with SINGLECAN
//#define SINGLECAN  // for testing (or non-Garmin) we can use one bus 
#define PICANM // v1 version on Tatiana

#ifdef SH_ESP32
#define POT_PIN 36 // SH_ESP32
#endif
#if defined(ESPBERRY) || defined(PICANM)
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

void WindSpeed(const tN2kMsg &N2kMsg);

extern tNMEA2000Handler NMEA2000Handlers[];

float readAnalogRotationValue();
void ParseWindCAN();
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
