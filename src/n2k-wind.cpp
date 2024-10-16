/*
n2k-wind.cpp
Here we will define part of the class that work only on the wind bus

Dual-ESP mast rotation solution
ESP1 (CYD) is either connected to 2 IMUs (one on mast, one in controller box), and a Hall-effect sensor
or connected to a Honeywell rotation sensor
ESP1 is connected to an isolated wind N2K bus to keep uncorrected wind data from Garmin displays
ESP1 corrects incoming wind data for rotation and forwards in Actisense format on UART
TBD: ESP1 stores AWA, AWS, and STW on SD card for polars
ESP2 (SH-ESP32) is connected to ESP1 via UART, receives correct wind data (Actisense), and transmits on main N2K bus
*/
#if defined(N2K) && defined (WINDBUS)
#include "compass.h"
#include "windparse.h"
#include "BoatData.h"

// object-oriented classes
#include "n2k.h"
#include "logto.h"

void ToggleLed();
void WindSpeed();

#define CAN_TX_PIN GPIO_NUM_22
#define CAN_RX_PIN GPIO_NUM_27

extern bool stackTrace;

extern tBoatData *pBD;
extern tBoatData BoatData;
// TBD: I should add mag heading, mast heading etc (all the globals) to BoatData struct so I could have a single extern in each file

tN2kMsg correctN2kMsg;
extern tN2kWindReference wRef;
extern tN2kHeadingReference hRef;
//HardwareSerial SerialPort(0);
//Stream *read_stream = &SerialPort;
//Stream *forward_stream = &SerialPort;
//#define RX 35 // white
//#define TX 21 // red
Stream *read_stream = &Serial;
Stream *forward_stream = &Serial;

tActisenseReader ActisenseReader;

bool ParseWindN2K(const tN2kMsg &N2kMsg);

void n2k::windCounter() {
    n2k::num_wind_messages++;
    total_time_since_last_wind += time_since_last_wind_rx;
    avg_time_since_last_wind = total_time_since_last_wind / n2k::num_wind_messages;
#ifdef WINDDIAG
    if (num_wind_messages % 100 == 0)
    {
        Serial.printf("last wind time: %2.2ld avg wind time: %2.2ld ms", time_since_last_wind_rx, avg_time_since_last_wind);
        if (time_since_last_wind_rx > 0.0)
            Serial.printf(" %2.2ld Hz", 1000.0 / avg_time_since_last_wind);
        Serial.println();
    }
#endif
    time_since_last_wind_rx = 0;
}

void n2k::mastCompCounter() {
    num_mastcomp_messages++;
    total_time_since_last_mastcomp += time_since_last_mastcomp_rx;
    avg_time_since_last_mastcomp = total_time_since_last_mastcomp / num_mastcomp_messages;
#ifdef MASTCOMPDIAG
    if (num_mastcomp_messages % 100 == 0)
    {
        Serial.printf("last mastcomp time: %2.2ld (secs) avg mastcomp time: %2.2ld ms", time_since_last_mastcomp_rx, avg_time_since_last_mastcomp);
        if (time_since_last_mastcomp_rx > 0.0)
            Serial.printf(" %2.2ld Hz", 1000.0 / avg_time_since_last_mastcomp);
        Serial.println();
    }
#endif
    time_since_last_mastcomp_rx = 0;
}

int compassDifference(int angle1, int angle2) {
    int diff = (angle1 - angle2 + 360) % 360;
    //Serial.print("compdiff: "); Serial.println(diff);
    return (diff > 180) ? (diff - 360) : diff;
}

float readCompassDelta() {
  if (imuReady) {
    float mastDelta = compassDifference(n2k::boatIMUdeg, n2k::mastIMUdeg+n2k::mastOrientation);
    //logTo::logToAll("mastDelta: " + String(mastDelta));
    mastAngle = mastDelta;
    //mastCompDelta.reading((int)(mastDelta*100)); // moving average
    return mastDelta;
  }
  //Serial.println("compass not ready");
  return -1;
}

//bool n2kWindOpen;

// NMEA 2000 message handler for wind bus
void n2k::HandleNMEA2000MsgWind(const tN2kMsg &N2kMsg) {
//void HandleNMEA2000MsgWind(const tN2kMsg &N2kMsg) {
    if (stackTrace)
        //N2kMsg.Print(&Serial);
        // maybe webserial?
    // Serial.printf("wind t: %d R: %d\n", millis(), N2kMsg.PGN);
    if (!n2kWindOpen) {
        n2kWindOpen = true;
        //n2kWind->SetForwardStream(FORWARD_STREAM);
    }
    ToggleLed();
    // problematic since mast compass can come back online
    // if (time_since_last_mastcomp_rx > 5000) // 5 second timeout on mast compass
    //  mastIMUdeg = -1;
    switch (N2kMsg.PGN) {
    case 130306L: {
        if (ParseWindN2K(N2kMsg))
            WindSpeed();
        break;
    }
    case 128259L: {
        //n2k::BoatSpeed(N2kMsg);
        break;
    }
    case 127250L: {
        // process heading (on wind bus so represents mast IMU)
        break;
    }
    }
}

bool ParseWindN2K(const tN2kMsg &N2kMsg) {
  unsigned char SID;  
  if (ParseN2kPGN130306(N2kMsg, SID, n2k::windSpeedMeters, n2k::windAngleRadians, wRef)) {
    n2k::windCounter();
    return true;
  } else {
    logTo::logToAll("no parse wind");
  }
  return false;
}

void WindSpeed() {
    n2k::windSpeedKnots = n2k::windSpeedMeters * 1.943844; // convert m/s to kts
    n2k::windAngleDegrees = n2k::windAngleRadians * RADTODEG;
    ////Serial.printf("sensor angle %0.2f\n", n2k::windAngleDegrees);
    if (wRef != N2kWind_Apparent) { // N2kWind_Apparent
        logTo::logToAll("got wind PGN not apparent!"  + String(wRef));
        return;
    }
    // read rotation value and correct
    float mastRotate = 0.0;
#ifdef BNO08XXX // not here any more because of weird issue with display not working
    if (compass.OnToggle) {
        mastRotate = readCompassDelta();
    }
#endif
    float anglesum = n2k::windAngleDegrees + mastRotate;    // sensor AFT of mast so subtract rotation
    // ensure sum is 0-359; rotateout holds the corrected AWA
    if (anglesum<0) {                             
        n2k::rotateout = anglesum + 360;
    } else if (anglesum>359) {   
        n2k::rotateout = anglesum - 360;               
    } else {
        n2k::rotateout = anglesum;               
    }
    #ifdef DEBUG
    logTo::logToAll("rotate now " + String(PotValue) + " low " + String(PotLo) + " hi " + String(PotHi));
    logTo::logToAll("mastrotate " + String(mastRotate) + " anglesum " + String(anglesum) + " rotateout " + String(rotateout));
    #endif
    // forward corrected wind on UART/Actisense
    // note we are sending the original speed reading in m/s
    // and the AWA converted from rads to degrees, corrected, and converted back to rads
    SetN2kPGN130306(correctN2kMsg, 0xFF, n2k::windSpeedMeters, n2k::rotateout*DEGTORAD, N2kWind_Apparent);
    correctN2kMsg.SendInActisenseFormat(forward_stream);
    logTo::logToAll("sending " + String(correctN2kMsg.PGN));
#if 0
    if (n2kMain->SendMsg(correctN2kMsg)) {
        ////Serial.printf("sent n2k wind %0.2f", rotateout);
    } else {
        logTo::logToAll("Failed to send wind");  
    }
#endif
    #ifdef XMITRUDDER
    // for now (until you dive into SensESP), send rotation angle as rudder
    SetN2kPGN127245(correctN2kMsg, (mastRotate+50)*DEGTORAD, 0, N2kRDO_NoDirectionOrder, 0);
    n2kMain->SendMsg(correctN2kMsg);
    #endif
    #ifdef XMITTRUE
    // calculate TWS/TWA from boat speed and send another wind PGN
    calcTrueWind();
    SetN2kPGN130306(correctN2kMsg, 0xFF, TWS, TWA, N2kWind_True_water);
    n2kMain->SendMsg(correctN2kMsg);
    #endif
}

// parse compass reading on wind bus
// this is a relative bearing from the mast
void ParseCompassN2K(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  double heading;
  double deviation;
  double variation;
  tN2kHeadingReference headingRef;
  //logTo::logToAll("parsecompassn2k");
  if (ParseN2kPGN127250(N2kMsg, SID, heading, deviation, variation, headingRef)) {
    // TBD get "reference" to confirm it's N2khr_Unavailable
    n2k::mastIMUdeg = heading * RADTODEG;
    readCompassDelta();
    // NOTE we do NOT transmit boat heading on N2K here; only from reaction in wind-bus.cpp, to avoid flooding bus
//#define XMITRUDDER
#ifdef XMITRUDDER // send rudder angle (as rudder #1)
    //SetN2kPGN127245(correctN2kMsg, heading, 1, N2kRDO_NoDirectionOrder, 0);
    SetN2kPGN127250(correctN2kMsg, 0xFF, (double)heading, N2kDoubleNA, N2kDoubleNA, N2khr_Unavailable);
    n2kMain->SendMsg(correctN2kMsg);
    logTo::logToAll("sent rudder " + String(heading) + " rad " + String(heading*DEGTORAD));
#endif
  }
}

// on wind bus we get all messages sent to SH-ESP32 on main bus to keep track and for display info
void HandleStreamN2kMsg(const tN2kMsg &N2kMsg) {
  N2kMsg.Print(&Serial);
  //n2k::n2kWind->SendMsg(N2kMsg,-1);
}

bool n2k::setupWindBus() {
    // read_stream and forward_stream
    //pinMode(TX, OUTPUT); // Initialize the pin as output
    //SerialPort.begin(BAUD, SERIAL_8N1, RX, TX);
    //Serial.begin(BAUD, SERIAL_8N1);
    logTo::logToAll("begin UART " + String(BAUD));

    n2kWind = new tNMEA2000_esp32(CAN_TX_PIN, CAN_RX_PIN);
    n2kWind->SetN2kCANMsgBufSize(250);
    n2kWind->SetN2kCANReceiveFrameBufSize(250);

    // Set Product information
    n2kWind->SetProductInformation(
        "20210334",  // Manufacturer's Model serial code (max 32 chars)
        104,         // Manufacturer's product code
        "SH-ESP32 NMEA 2000 USB GW",  // Manufacturer's Model ID (max 33 chars)
        "0.1.0.0 (2021-03-31)",  // Manufacturer's Software version code (max 40
                                // chars)
        "0.0.3.1 (2021-03-07)"   // Manufacturer's Model version (max 24 chars)
    );
    // Set device information
    n2kWind->SetDeviceInformation(
        666,    // Unique number. Use e.g. Serial number.
        130,  // Device function=Analog to NMEA 2000 Gateway. See codes on
                // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
        25,   // Device class=Inter/Intranetwork Device. See codes on
            // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
        2047  // Just choosen free from code list on
                // http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
    );

    n2kWind->SetMode(tNMEA2000::N2km_ListenAndNode);
    //n2kWind->SetForwardType(tNMEA2000::fwdt_Text);
    // n2kWind->EnableForward(false);
    //n2kWind->SetForwardOwnMessages(true);
    //n2kWind->SetForwardStream(forward_stream);

    n2kWind->SetMsgHandler(HandleNMEA2000MsgWind);
    if (n2kWind->Open()) {
        logTo::logToAll("opening n2kWind");
        return true;
    }
    else {
        logTo::logToAll("failed to open n2kWind");
        return false;
    }
    ActisenseReader.SetReadStream(read_stream);
    ActisenseReader.SetDefaultSource(75);
    ActisenseReader.SetMsgHandler(HandleStreamN2kMsg); 
}

#endif // N2K
