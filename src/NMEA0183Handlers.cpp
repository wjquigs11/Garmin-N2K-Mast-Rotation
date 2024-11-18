 

#include "NMEA0183Handlers.h"
#include "BoatData.h"

tBoatData *pBD=0;

#ifdef NMEA0183
#include <N2kMsg.h>
#include <NMEA2000.h>
#include <N2kMessages.h>
#include <NMEA0183Messages.h>

extern int num_0183_messages;
extern int num_0183_fail;
extern int num_0183_ok;

struct tNMEA0183Handler {
  const char *Code;
  void (*Handler)(const tNMEA0183Msg &NMEA0183Msg); 
  int numMessages;
};

// IC-M330 output:
// RMC $GPRMC	Time, date, position, course, speed data.
// GSA $GPGSA	GPS receiver operating mode, satellites used in the position solution, DOP values.
// GSV $GPGSV	Number of satellites in view, satellite ID numbers, elevation, azimuth, SNR values.
//
// old bridge sentences:
// 129026: COGSOGRapid 
// 129029: GNSS Position Data 
// 129540: SatelliteInfo 
// 127258: Magnetic Variation 
// 129025: SetLatLonRapid 

// Predefinition for functions to make it possible for constant definition for NMEA0183Handlers
void HandleRMC(const tNMEA0183Msg &NMEA0183Msg);
//void HandleGSA(const tNMEA0183Msg &NMEA0183Msg);
void HandleGSV(const tNMEA0183Msg &NMEA0183Msg);

// Internal variables
extern tNMEA2000 *n2kMain;
Stream* NMEA0183HandlersDebugStream=0;
struct tGSV SatInfo[4];
//struct tSatelliteInfo[MaxSatelliteInfoCount];
//struct tSatelliteInfo[8];

tNMEA0183Handler NMEA0183Handlers[]={
  {"RMC",&HandleRMC, 0}, // Position, Velocity, and Time
  //{"GSA",&HandleGSA}, // GPS DOP and active satellites // not doing this one yet because no parser
  //{"GSV",&HandleGSV}, // Number of SVs in view, PRN, elevation, azimuth, and SNR
  {0,0}
};

void DebugNMEA0183Handlers(Stream* _stream) {
  NMEA0183HandlersDebugStream=_stream;
}

tN2kGNSSmethod GNSMethofNMEA0183ToN2k(int Method) {
  switch (Method) {
    case 0: return N2kGNSSm_noGNSS;
    case 1: return N2kGNSSm_GNSSfix;
    case 2: return N2kGNSSm_DGNSS;
    default: return N2kGNSSm_noGNSS;  
  }
}

void HandleNMEA0183Msg(const tNMEA0183Msg &NMEA0183Msg) {
  num_0183_messages++;
  int iHandler;
  // Find handler
  for (iHandler=0; NMEA0183Handlers[iHandler].Code!=0 && !NMEA0183Msg.IsMessageCode(NMEA0183Handlers[iHandler].Code); iHandler++);
  if (NMEA0183Handlers[iHandler].Code!=0) {
    NMEA0183Handlers[iHandler].numMessages++;
    NMEA0183Handlers[iHandler].Handler(NMEA0183Msg); 
  }
}

// NMEA0183 message Handler functions

void HandleRMC(const tNMEA0183Msg &NMEA0183Msg) {
  if (pBD==0) return;
  
  double variation;
  if (NMEA0183ParseRMC_nc(NMEA0183Msg,pBD->GPSTime,pBD->Latitude,pBD->Longitude,pBD->COG,pBD->SOG,pBD->DaysSince1970,variation)) {
  } else if (NMEA0183HandlersDebugStream!=0) { NMEA0183HandlersDebugStream->println("Failed to parse RMC"); }
  if (n2kMain!=0) {
      tN2kMsg N2kMsg;
      // COGSOGRapid
      SetN2kPGN129026(N2kMsg, 255, N2khr_true, pBD->COG, pBD->SOG);
      if (!n2kMain->SendMsg(N2kMsg)) num_0183_fail++; else num_0183_ok++;
      // GNSSPosition
      SetN2kPGN129029(N2kMsg,255,pBD->DaysSince1970,pBD->GPSTime,
                  pBD->Latitude,pBD->Longitude,pBD->Altitude,
                  N2kGNSSt_GPS,N2kGNSSm_GNSSfix,
                  pBD->SatelliteCount,pBD->HDOP,0,0,
                  0,N2kGNSSt_GPS,0,0);
      if (!n2kMain->SendMsg(N2kMsg)) num_0183_fail++; else num_0183_ok++;
      // LatLonRapid
      SetN2kPGN129025(N2kMsg, pBD->Latitude, pBD->Longitude);
      if (!n2kMain->SendMsg(N2kMsg)) num_0183_fail++; else num_0183_ok++;
      // Variation - taking out since it doesn't seem to be parsing correctly
      //SetN2kPGN127258(N2kMsg, 255, N2kmagvar_Calc, pBD->DaysSince1970, pBD->Variation);
      //n2kMain->SendMsg(N2kMsg);
    }
  if (NMEA0183HandlersDebugStream!=0) {
      NMEA0183HandlersDebugStream->print("RMC Time="); NMEA0183HandlersDebugStream->println(pBD->GPSTime);
      NMEA0183HandlersDebugStream->print("Latitude="); NMEA0183HandlersDebugStream->println(pBD->Latitude,5);
      NMEA0183HandlersDebugStream->print("Longitude="); NMEA0183HandlersDebugStream->println(pBD->Longitude,5);
      NMEA0183HandlersDebugStream->print("COG="); NMEA0183HandlersDebugStream->println(pBD->COG);
      NMEA0183HandlersDebugStream->print("SOG="); NMEA0183HandlersDebugStream->println(pBD->SOG);
      NMEA0183HandlersDebugStream->print("Variation="); NMEA0183HandlersDebugStream->println(pBD->Variation);  
  }
}

// We might get a sequence of GSV messages, each with up to 4 satellites
// Keep a running count
static int SatCount=0;
static int totalMSG=0;

void HandleGSV(const tNMEA0183Msg &NMEA0183Msg) {
  int thisMSG;
  if (pBD==0) return;
  /*
  if (NMEA0183ParseGSV_nc(NMEA0183Msg, totalMSG, thisMSG, pBD->SatelliteCount, &SatInfo[0], &SatInfo[1], &SatInfo[2], &SatInfo[3]) {}
  } else if (NMEA0183HandlersDebugStream!=0) { NMEA0183HandlersDebugStream->println("Failed to parse GSV"); }
  // if this isn't the last message in the sequence, set up more satellites
  if (n2kMain!=0) {
      if (totalSat == totalMSG) {
        // we've collected all satellites, now build message and send
          tN2kMsg N2kMsg;
      SetN2kPGN129540(N2kMsg, 255, N2kDD072_RangeResidualsWereUsedToCalculateData);
      if (pBD->SatelliteCount > 4) pBD->SatelliteCount=4;
      for (int i=1; i<pBD->SatelliteCount; i++) {
    // might as well send "rapid" PGNs again
        SetN2kCOGSOGRapid(N2kMsg, 255, N2khr_true, pBD->COG, pBD->SOG);
        n2kMain->SendMsg(N2kMsg); 
        SetN2kLatLonRapid(N2kMsg, pBD->Latitude, pBD->Longitude);
        n2kMain->SendMsg(N2kMsg); 
    }
  }
  if (NMEA0183HandlersDebugStream!=0) {
      NMEA0183HandlersDebugStream->print("totalMSG="); NMEA0183HandlersDebugStream->println(totalMSG);
      NMEA0183HandlersDebugStream->print("thisMSG"); NMEA0183HandlersDebugStream->println(thisMSG);
      NMEA0183HandlersDebugStream->print("SatCount="); NMEA0183HandlersDebugStream->println(pBD->SatelliteCount);
      NMEA0183HandlersDebugStream->print("Sat1"); NMEA0183HandlersDebugStream->println(Sat1.SVID);
  }
      */
}
#endif

