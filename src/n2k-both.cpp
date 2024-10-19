/*
n2k-both.cpp
Here we will initialize the class (since it must all be static because it's a "singleton")
And define general functionst that may be used by either wind or main bus
*/
#ifdef N2K
#include "compass.h"
#include "windparse.h"
#include "BoatData.h"

// object-oriented classes
#include "n2k.h"
#include "logto.h"

void WindSpeed();

#define CAN_TX_PIN GPIO_NUM_27
#define CAN_RX_PIN GPIO_NUM_22

const unsigned long TransmitMessages[] PROGMEM = {130306L, 127250L, 0};
tNMEA2000* n2k::n2kMain = nullptr;
bool n2k::n2kMainOpen = false;
tNMEA2000* n2k::n2kWind = nullptr;
bool n2k::n2kWindOpen = false;
float n2k::rotateout = 0.0f;
int n2k::mastAngle = 0;
int n2k::num_n2k_recv = 0;
int n2k::num_n2k_xmit = 0;
int n2k::num_wind_recv = 0;
int n2k::num_wind_xmit = 0;
int n2k::num_wind_fail=0;
int n2k::num_wind_other=0;
int n2k::num_wind_other_fail=0;
int n2k::num_wind_other_ok=0;
int n2k::num_mastIMU_messages=0;
elapsedMillis n2k::time_since_last_can_rx=0;
elapsedMillis n2k::time_since_last_wind_rx=0;
unsigned long n2k::total_time_since_last_wind=0;
unsigned long n2k::avg_time_since_last_wind=0;
elapsedMillis n2k::time_since_last_mastIMU_rx=0;
unsigned long n2k::total_time_since_last_mastIMU=0;
unsigned long n2k::avg_time_since_last_mastIMU=0;
double n2k::windSpeedKnots=0;
double n2k::windSpeedMeters=0;
double n2k::windAngleDegrees=0;
double n2k::windAngleRadians=0;
int n2k::mastOrientation=0; // delta between mast compass and boat compass
int n2k::boatOrientation=0; // delta between boat compass and magnetic north
float n2k::boatCompassTrue=0;
float n2k::mastIMUdeg=0;
float n2k::boatIMUdeg=0;
float n2k::mastDelta=0;
unsigned long n2k::otherPGN[MAXPGN];
int n2k::otherPGNindex=0;
String n2k::can_state;

// using namespace reactesp;
// extern ReactESP app;
extern bool stackTrace;

extern tBoatData *pBD;
extern tBoatData BoatData;

// TBD: I should add mag heading, mast heading etc (all the globals) to BoatData struct so I could have a single extern in each file

tN2kWindReference wRef;
tN2kHeadingReference hRef;


void n2k::ToggleLed() {
  static bool led_state = false;
  digitalWrite(LED_BUILTIN, led_state);
  led_state = !led_state;
}

void n2k::RecoverFromCANBusOff() {
    if (stackTrace)
        Serial.println("RecoverFromCANBusOff");

    // This recovery routine first discussed in
    // https://www.esp32.com/viewtopic.php?t=5010 and also implemented in
    // https://github.com/wellenvogel/esp32-nmea2000
    static bool recovery_in_progress = false;
    static elapsedMillis recovery_timer;
    if (recovery_in_progress && recovery_timer < RECOVERY_RETRY_MS)
    {
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

void n2k::PollCANStatus() {
    // CAN controller registers are SJA1000 compatible.
    // Bus status value 0 indicates bus-on; value 1 indicates bus-off.
    unsigned int bus_status = MODULE_CAN->SR.B.BS;

    switch (bus_status)
    {
    case 0:
        can_state = "RUN";
        break;
    case 1:
        can_state = "OFF";
        // try to automatically recover
        // RecoverFromCANBusOff();
        break;
    }
}

#endif // N2K
