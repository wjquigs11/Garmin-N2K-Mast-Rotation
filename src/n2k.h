// n2k.h class definition

#ifndef N2K_H
#define N2K_H

#include <N2kMessages.h>
#include <NMEA2000_esp32.h>
#include <ActisenseReader.h>
#include <elapsedMillis.h>

#define RECOVERY_RETRY_MS 1000  // How long to attempt CAN bus recovery

// ESP1 reads compass from ESP2, sends corrected wind
// ESP2 reads wind from ESP1...

class n2k {
private:

public:  
    static Stream *Serial1;
    static Stream *ForwardStream;

    static tNMEA2000 *n2kMain;
    static bool n2kMainOpen;
    static tNMEA2000 *n2kWind;
    static bool n2kWindOpen;
    static float rotateout;

    // timing/display
    static int num_n2k_messages;
    static int num_wind_messages;
    static int num_wind_fail;
    static int num_wind_other;
    static int num_wind_other_fail;
    static int num_wind_other_ok;
    static int num_mastcomp_messages;
    static elapsedMillis time_since_last_can_rx;
    static elapsedMillis time_since_last_wind_rx;
    static unsigned long total_time_since_last_wind;
    static unsigned long avg_time_since_last_wind;
    static elapsedMillis time_since_last_mastcomp_rx;
    static unsigned long total_time_since_last_mastcomp;
    static unsigned long avg_time_since_last_mastcomp;

    static double windSpeedKnots;
    static double windSpeedMeters;
    static double windAngleDegrees;
    static double windAngleRadians;

    static int mastOrientation; // delta between mast compass and boat compass
    static int boatOrientation; // delta between boat compass and magnetic north
    static float boatIMUdeg; // magnetic heading not corrected for variation
    static float boatCompassTrue;
    static float mastIMUdeg;
    static float mastDelta;

    static unsigned long otherPGN[];
    static int otherPGNindex;

    static void ToggleLed();
    static void ParseCompassN2K(const tN2kMsg &N2kMsg);
    static void BoatSpeed(const tN2kMsg &N2kMsg);
    static void windCounter();
    static void mastCompCounter();
    static void HandleNMEA2000MsgMain(const tN2kMsg &N2kMsg);
    static void HandleNMEA2000MsgWind(const tN2kMsg &N2kMsg);
    static void RecoverFromCANBusOff();
    static void PollCANStatus();
    static bool setupMainBus();
    static bool setupWindBus();
};

#endif
