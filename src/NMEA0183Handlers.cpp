 
#include "BoatData.h"
#include "NMEA0183Handlers.h"
#include "windparse.h"
#include "logto.h"

tBoatData *pBD=0;
tRTKstats *pRTK=0;
tEnvStats *pENV=0; // shouldn't really be here but it's fukit friday!

#ifdef NMEA0183

unsigned int num_0183_messages;
unsigned int num_0183_fail;
unsigned int num_0183_ok;

bool debugNMEA = false;

// WITmotion RTK output:
// $GPGSV,3,1,10,22,76,043,20,17,75,121,22,14,55,074,25,19,53,194,24,1*60 (multi-part)
// $GNGGA,010709.000,4740.262183,N,12219.447475,W,1,17,1.12,91.5,M,-17.3,M,,*7D
// $GNRMC,010709.000,A,4740.262183,N,12219.44
// $GLGSV,1,1,03,79,70,027,14,81,50,131,17,78,29,094,25,1*45
// $GAGSV,2,1,07,25,81,345,22,03,38,235,15,24,36,076,20,08,32,301,17,7*7E
// $GBGSV,2,1,07,24,80,092,21,44,57,123,16,12,44,160,15,25,42,297,15,1*78
// $GQGSV,1,1,00,1*65
// $GNGSA,A,3,22,17,14,19,30,02,21,,,,,,1.39,1.12,0.82,1*0B
// $GNGLL,4740.262183,N,12219.447475,W,010709.000,A,A*59
// $GNVTG,242.46,T,,M,0.078,N,0.144,K,A*2B
// $PQTMANTENNASTATUS,2,2,2,2,2*4D
// $PQTMTAR,1,010709.000,0,,0.000,1.985139,-0.924438,,0.003247,0.003242,,00*47

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
void HandleGGA(const tNMEA0183Msg &NMEA0183Msg);
void HandleGSV(const tNMEA0183Msg &NMEA0183Msg);
void HandleGSA(const tNMEA0183Msg &NMEA0183Msg);
void HandleVTG(const tNMEA0183Msg &NMEA0183Msg);
void HandleGLL(const tNMEA0183Msg &NMEA0183Msg);
void HandlePQTMANTEN(const tNMEA0183Msg &NMEA0183Msg);
void HandlePQTMTAR(const tNMEA0183Msg &NMEA0183Msg);

// Internal variables
tNMEA2000 *NMEA2000;
Stream* NMEA0183HandlersDebugStream=0;
struct tGSV SatInfo[5];
//struct tSatelliteInfo[MaxSatelliteInfoCount];
//struct tSatelliteInfo[8];

tNMEA0183Handler NMEA0183Handlers[]={
  {"RMC",&HandleRMC, 0}, // Position, Velocity, and Time
  {"GSV",&HandleGSV, 0}, // Number of SVs in view, PRN, elevation, azimuth, and SNR
  {"GGA",&HandleGGA, 0}, // UTC Time, Latitude, Direction of Latitude, Longitude, Direction of Longitude, GPS Quality Indicator, Number of Satellites in Use, HDOP, Altitude, Unit of Altitude, Geoidal Separation, Unit of Geoidal Separation, Age of Differential GPS Data, Differential Reference Station ID
  {"GSA",&HandleGSA, 0}, // GPS DOP and active satellites // not doing this one yet because no parser
  {"VTG",&HandleVTG, 0}, // track made good (course over ground) and the speed over ground
  {"GLL",&HandleGLL, 0}, // GPS lat lon
  {"TMANTENNASTATUS",&HandlePQTMANTEN, 0}, // RTK antenna status
  {"TMTAR",&HandlePQTMTAR, 0}, // time and attitude (heading)
  {0,0,0}
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

char msg[128];

void HandleNMEA0183Msg(const tNMEA0183Msg &NMEA0183Msg) {
  num_0183_messages++;
  //logTo::logToAll(String(num_0183_messages) + " " + String(NMEA0183Msg.Sender()) + String(NMEA0183Msg.MessageCode()));
  int iHandler;
  //bool found;
  // Find handler
  //logTo::logToAll(String(NMEA0183Msg.MessageCode()));
  for (iHandler=0; NMEA0183Handlers[iHandler].Code!=0 && !NMEA0183Msg.IsMessageCode(NMEA0183Handlers[iHandler].Code); iHandler++);
  if (NMEA0183Handlers[iHandler].Code!=0) {
    //found = true;
    NMEA0183Handlers[iHandler].numMessages++;
    //logTo::logToAll(String(NMEA0183Handlers[iHandler].Code) + " " + String(NMEA0183Handlers[iHandler].numMessages));
    NMEA0183Handlers[iHandler].Handler(NMEA0183Msg); 
  } else {
  //if (!found) {
    //logTo::logToAll(String(NMEA0183Msg.Sender()) + String(NMEA0183Msg.MessageCode()));
    Serial.printf("%s%s",NMEA0183Msg.Sender(),NMEA0183Msg.MessageCode());
    for (int i=0; i < NMEA0183Msg.FieldCount(); i++) {
      //logTo::logToAll(String(NMEA0183Msg.Field(i)));
      Serial.print(NMEA0183Msg.Field(i));
      if ( i<NMEA0183Msg.FieldCount()-1 ) Serial.print(" ");
    }
    Serial.println();
    num_0183_ok++;
  }
}

// NMEA0183 message Handler functions

void HandleRMC(const tNMEA0183Msg &NMEA0183Msg) {
  if (pBD==0) return;
  
  double variation;
  if (NMEA0183ParseRMC_nc(NMEA0183Msg,pBD->GPSTime,pBD->Latitude,pBD->Longitude,pBD->COG,pBD->SOG,pBD->DaysSince1970,variation)) {
  } else if (NMEA0183HandlersDebugStream!=0) { NMEA0183HandlersDebugStream->println("Failed to parse RMC"); }
  if (NMEA2000!=0) {
      tN2kMsg N2kMsg;
      // COGSOGRapid
      SetN2kPGN129026(N2kMsg, 255, N2khr_true, pBD->COG, pBD->SOG);
      if (!NMEA2000->SendMsg(N2kMsg)) num_0183_fail++; else num_0183_ok++;
      // GNSSPosition
      SetN2kPGN129029(N2kMsg,255,pBD->DaysSince1970,pBD->GPSTime,
                  pBD->Latitude,pBD->Longitude,pBD->Altitude,
                  N2kGNSSt_GPS,N2kGNSSm_GNSSfix,
                  pBD->SatelliteCount,pBD->HDOP,0,0,
                  0,N2kGNSSt_GPS,0,0);
      if (!NMEA2000->SendMsg(N2kMsg)) num_0183_fail++; else num_0183_ok++;
      // LatLonRapid
      SetN2kPGN129025(N2kMsg, pBD->Latitude, pBD->Longitude);
      if (!NMEA2000->SendMsg(N2kMsg)) num_0183_fail++; else num_0183_ok++;
      // Variation - taking out since it doesn't seem to be parsing correctly
      //SetN2kPGN127258(N2kMsg, 255, N2kmagvar_Calc, pBD->DaysSince1970, pBD->Variation);
      //NMEA2000->SendMsg(N2kMsg);
    }
  if (NMEA0183HandlersDebugStream!=0) {
      NMEA0183HandlersDebugStream->print("RMC Time="); NMEA0183HandlersDebugStream->println(pBD->GPSTime);
      NMEA0183HandlersDebugStream->print("Latitude="); NMEA0183HandlersDebugStream->println(pBD->Latitude,5);
      NMEA0183HandlersDebugStream->print("Longitude="); NMEA0183HandlersDebugStream->println(pBD->Longitude,5);
      NMEA0183HandlersDebugStream->print("COG="); NMEA0183HandlersDebugStream->println(pBD->COG);
      NMEA0183HandlersDebugStream->print("SOG="); NMEA0183HandlersDebugStream->println(pBD->SOG);
      NMEA0183HandlersDebugStream->print("Variation="); NMEA0183HandlersDebugStream->println(pBD->Variation);  
  }
  //logTo::logToAll("RMC lat: " + String(pBD->Latitude,5) + " lon: " + String(pBD->Longitude,5) + " COG: " + String(pBD->COG));
}

// We might get a sequence of GSV messages, each with up to 4 satellites
// Keep a running count
static int SatCount=0;
static int totalMSG=0;

void HandleGSV(const tNMEA0183Msg &NMEA0183Msg) {
  return;
  int thisMSG;
  int SatelliteCount;
  if (pBD==0) return;
  if (NMEA0183ParseGSV_nc(NMEA0183Msg, totalMSG, thisMSG, SatelliteCount, SatInfo[0], SatInfo[1], SatInfo[2], SatInfo[3])) {
  } else if (NMEA0183HandlersDebugStream!=0) { NMEA0183HandlersDebugStream->println("Failed to parse GSV"); }
  if (SatelliteCount != pBD->SatelliteCount) {
    // not working because satcount is PER MESSAGE
    logTo::logToAll("GSV sat count changed old: " + String(pBD->SatelliteCount) + " new: " + String(SatelliteCount));
    pBD->SatelliteCount = SatelliteCount;
  }
#if 0
  // if this isn't the last message in the sequence, set up more satellites
  if (NMEA2000!=0) {
      if (SatCount == totalMSG) {
        // we've collected all satellites, now build message and send
          tN2kMsg N2kMsg;
      SetN2kPGN129540(N2kMsg, 255, N2kDD072_RangeResidualsWereUsedToCalculateData);
      if (pBD->SatelliteCount > 4) pBD->SatelliteCount=4;
      for (int i=1; i<pBD->SatelliteCount; i++) {
    // might as well send "rapid" PGNs again
        SetN2kCOGSOGRapid(N2kMsg, 255, N2khr_true, pBD->COG, pBD->SOG);
        NMEA2000->SendMsg(N2kMsg); 
        SetN2kLatLonRapid(N2kMsg, pBD->Latitude, pBD->Longitude);
        NMEA2000->SendMsg(N2kMsg); 
      }
    }
  }
  if (NMEA0183HandlersDebugStream!=0) {
      NMEA0183HandlersDebugStream->print("totalMSG="); NMEA0183HandlersDebugStream->println(totalMSG);
      NMEA0183HandlersDebugStream->print("thisMSG"); NMEA0183HandlersDebugStream->println(thisMSG);
      NMEA0183HandlersDebugStream->print("SatCount="); NMEA0183HandlersDebugStream->println(pBD->SatelliteCount);
      NMEA0183HandlersDebugStream->print("Sat1"); NMEA0183HandlersDebugStream->println(SatInfo[0].SVID);
  }
#endif
}

void HandleGSA(const tNMEA0183Msg &NMEA0183Msg) {
  return;
}

void HandleGGA(const tNMEA0183Msg &NMEA0183Msg) {
  int SatelliteCount;
  if (pBD==0) return;
  if (NMEA0183ParseGGA_nc(NMEA0183Msg,pBD->GPSTime,pBD->Latitude,pBD->Longitude,
                   pBD->GPSQualityIndicator,SatelliteCount,pBD->HDOP,pBD->Altitude,pBD->GeoidalSeparation,
                   pBD->DGPSAge,pBD->DGPSReferenceStationID)) {
    } else if (NMEA0183HandlersDebugStream!=0) { NMEA0183HandlersDebugStream->println("Failed to parse GGA"); }
  if (abs(SatelliteCount-pBD->SatelliteCount) > 1) {
    logTo::logToAll("GGA sat count changed old: " + String(pBD->SatelliteCount) + " new: " + String(SatelliteCount));
    pBD->SatelliteCount = SatelliteCount;
  }
  if (NMEA2000!=0) {
      tN2kMsg N2kMsg;
      SetN2kGNSS(N2kMsg,1,pBD->DaysSince1970,pBD->GPSTime,pBD->Latitude,pBD->Longitude,pBD->Altitude,
                N2kGNSSt_GPS,GNSMethofNMEA0183ToN2k(pBD->GPSQualityIndicator),pBD->SatelliteCount,pBD->HDOP,0,
                pBD->GeoidalSeparation,1,N2kGNSSt_GPS,pBD->DGPSReferenceStationID,pBD->DGPSAge
                );
      NMEA2000->SendMsg(N2kMsg); 
    }

    if (NMEA0183HandlersDebugStream!=0) {
      NMEA0183HandlersDebugStream->print("Time="); NMEA0183HandlersDebugStream->println(pBD->GPSTime);
      NMEA0183HandlersDebugStream->print("Latitude="); NMEA0183HandlersDebugStream->println(pBD->Latitude,5);
      NMEA0183HandlersDebugStream->print("Longitude="); NMEA0183HandlersDebugStream->println(pBD->Longitude,5);
      NMEA0183HandlersDebugStream->print("Altitude="); NMEA0183HandlersDebugStream->println(pBD->Altitude,1);
      NMEA0183HandlersDebugStream->print("GPSQualityIndicator="); NMEA0183HandlersDebugStream->println(pBD->GPSQualityIndicator);
      NMEA0183HandlersDebugStream->print("SatelliteCount="); NMEA0183HandlersDebugStream->println(pBD->SatelliteCount);
      NMEA0183HandlersDebugStream->print("HDOP="); NMEA0183HandlersDebugStream->println(pBD->HDOP);
      NMEA0183HandlersDebugStream->print("GeoidalSeparation="); NMEA0183HandlersDebugStream->println(pBD->GeoidalSeparation);
      NMEA0183HandlersDebugStream->print("DGPSAge="); NMEA0183HandlersDebugStream->println(pBD->DGPSAge);
      NMEA0183HandlersDebugStream->print("DGPSReferenceStationID="); NMEA0183HandlersDebugStream->println(pBD->DGPSReferenceStationID);
    }
    //logTo::logToAll("GGA lat: " + String(pBD->Latitude,5) + " lon: " + String(pBD->Longitude,5) + " qual: " + String(pBD->GPSQualityIndicator) + " sat: " + String(pBD->SatelliteCount));
}

void HandleVTG(const tNMEA0183Msg &NMEA0183Msg) {
 double MagneticCOG;
  if (pBD==0) return;
  
  if (NMEA0183ParseVTG_nc(NMEA0183Msg,pBD->COG,MagneticCOG,pBD->SOG)) {
    //pBD->Variation=pBD->COG-MagneticCOG; // Save variation for Magnetic heading
  } else if (NMEA0183HandlersDebugStream!=0) { NMEA0183HandlersDebugStream->println("Failed to parse VTG"); }
  if (NMEA2000!=0) { 
    tN2kMsg N2kMsg;
    SetN2kCOGSOGRapid(N2kMsg,1,N2khr_true,pBD->COG,pBD->SOG);
    NMEA2000->SendMsg(N2kMsg);
//      SetN2kBoatSpeed(N2kMsg,1,SOG);
//      NMEA2000.SendMsg(N2kMsg);
  }
  if (NMEA0183HandlersDebugStream!=0) {
    NMEA0183HandlersDebugStream->print("True heading="); NMEA0183HandlersDebugStream->println(pBD->TrueHeading);
  }
}

void HandleGLL(const tNMEA0183Msg &NMEA0183Msg) {
  return;
}

double NMEA0183GetDouble(const char *data);

// Quectel status of antennas
void HandlePQTMANTEN(const tNMEA0183Msg &NMEA0183Msg) {
  bool result=(NMEA0183Msg.FieldCount()>=5);
  if (result) {
    double Version = NMEA0183GetDouble(NMEA0183Msg.Field(0));
    pRTK->antennaAstat = NMEA0183GetDouble(NMEA0183Msg.Field(1));
    double reservedA = NMEA0183GetDouble(NMEA0183Msg.Field(2));
    pRTK->antennaBstat = NMEA0183GetDouble(NMEA0183Msg.Field(3));
    double reservedB = NMEA0183GetDouble(NMEA0183Msg.Field(4));
    //Serial.printf("PQTMANTEN v: %f a: %f r: %f b: %f r: %f\n", Version, AntAstatus, reservedA, AntBstatus, reservedB);
    //GNSS antenna A status. 
    //0 = Unknown
    //1 = Normal
    //2 = Open circuit
    //3 = Short circuit
  }
}

void HandlePQTMTAR(const tNMEA0183Msg &NMEA0183Msg) {
  if (debugNMEA) {
    String logMsg = "PQTMTAR: ";
    for (int i=0; i<NMEA0183Msg.FieldCount(); i++) {
      logMsg += String(NMEA0183Msg.Field(i)) + ",";
    }
    logTo::logToAll(logMsg);
  }
  bool result=(NMEA0183Msg.FieldCount()>=12);
  if (result) {
    double Version = NMEA0183GetDouble(NMEA0183Msg.Field(0));
    pRTK->GPStime = NMEA0183GPTimeToSeconds(NMEA0183Msg.Field(1)); // format?
    pRTK->RTKqual = (int)NMEA0183GetDouble(NMEA0183Msg.Field(2));
    // GNSS heading status indication:
    // 0 = Not available or invalid
    // 4 = RTK mode with fixed integers
    // 6 = Estimated (dead reckoning) mode
    double Reserved = NMEA0183GetDouble(NMEA0183Msg.Field(3));
    pRTK->baseLen = NMEA0183GetDouble(NMEA0183Msg.Field(4));  // base line
    pRTK->pitch = NMEA0183GetDouble(NMEA0183Msg.Field(5));
    pRTK->roll = NMEA0183GetDouble(NMEA0183Msg.Field(6));
    pRTK->heading = NMEA0183GetDouble(NMEA0183Msg.Field(7));
    pRTK->pAcc = NMEA0183GetDouble(NMEA0183Msg.Field(8)); // accuracy
    pRTK->rAcc = NMEA0183GetDouble(NMEA0183Msg.Field(9)); // accuracy
    pRTK->hAcc = NMEA0183GetDouble(NMEA0183Msg.Field(10)); // accuracy
    pRTK->usedSV = NMEA0183GetDouble(NMEA0183Msg.Field(12)); // number of satellites
    //Serial.printf("v: %f t: %f q: %f r: %f l: %f p: %f r: %f h: %f pa: %f ra: %f ha: %f sv: %f\n",
    //  Version, GPStime, Quality, Reserved, Length, Pitch, Roll, Heading, PitchAcc, RollAcc, HeadAcc, UsedSV);
    //logTo::logToAll("PQTMTAR q: " + String(Quality) + " l: " + String(Length) + " p: " + String(Pitch) + " r: " + String(Roll) + " h: " + String(Roll) + " sv: " + String(UsedSV));
  }
}

#endif
