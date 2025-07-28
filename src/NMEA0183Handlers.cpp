 
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
// $GLGSV,1,1,03,79,70,027,14,81,50,131,17,78,29,094,25,1*45 // Glonass
// $GAGSV,2,1,07,25,81,345,22,03,38,235,15,24,36,076,20,08,32,301,17,7*7E // Galileo
// $GBGSV,2,1,07,24,80,092,21,44,57,123,16,12,44,160,15,25,42,297,15,1*78 // Beidou
// $GQGSV,1,1,00,1*65 // QZSS
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
void HandleGGA(const tNMEA0183Msg &NMEA0183Msg);
void HandleGSV(const tNMEA0183Msg &NMEA0183Msg);
void HandleGSA(const tNMEA0183Msg &NMEA0183Msg);
void HandleVTG(const tNMEA0183Msg &NMEA0183Msg);
void HandleGLL(const tNMEA0183Msg &NMEA0183Msg);
void HandlePQTMANTEN(const tNMEA0183Msg &NMEA0183Msg);
void HandlePQTMTAR(const tNMEA0183Msg &NMEA0183Msg);
void HandleHPR(const tNMEA0183Msg &NMEA0183Msg);
void HandleVTG(const tNMEA0183Msg &NMEA0183Msg);

// Internal variables
//tNMEA2000 *NMEA2000;
Stream* NMEA0183HandlersDebugStream=0;
struct tGSV SatInfo[4];
struct tGSV GSVseen[MAXSAT];  
//struct tSatelliteInfo[MaxSatelliteInfoCount];
//struct tSatelliteInfo[8];

tNMEA0183Handler NMEA0183Handlers[]={
  {"RMC",&HandleRMC, 0}, // Position, Velocity, and Time
  {"GSV",&HandleGSV, 0}, // Number of SVs in view, PRN, elevation, azimuth, and SNR
  {"GGA",&HandleGGA, 0}, // UTC Time, Latitude, Direction of Latitude, Longitude, Direction of Longitude, GPS Quality Indicator, Number of Satellites in Use, HDOP, Altitude, Unit of Altitude, Geoidal Separation, Unit of Geoidal Separation, Age of Differential GPS Data, Differential Reference Station ID
  {"GSA",&HandleGSA, 0}, // GPS DOP and active satellites // not doing this one yet because no parser
  {"VTG",&HandleVTG, 0}, // track made good (course over ground) and the speed over ground
  {"GLL",&HandleGLL, 0}, // GPS lat lon
  {"VTG",&HandleVTG, 0}, // track made good (course over ground) and the speed over ground
  // UM982
  {"HPR",&HandleHPR, 0}, // Heading, pitch, roll
#ifdef WITMOTION
  {"TMANTENNASTATUS",&HandlePQTMANTEN, 0}, // RTK antenna status
  {"TMTAR",&HandlePQTMTAR, 0}, // time and attitude (heading)
#endif
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
  //log::toAll(String(num_0183_messages) + " " + String(NMEA0183Msg.Sender()) + String(NMEA0183Msg.MessageCode()));
  int iHandler;
  // Find handler
  //log::toAll(String(NMEA0183Msg.MessageCode()));
  for (iHandler=0; NMEA0183Handlers[iHandler].Code!=0 && !NMEA0183Msg.IsMessageCode(NMEA0183Handlers[iHandler].Code); iHandler++);
  if (NMEA0183Handlers[iHandler].Code!=0) {
    NMEA0183Handlers[iHandler].numMessages++;
    //log::toAll(String(NMEA0183Handlers[iHandler].Code) + " " + String(NMEA0183Handlers[iHandler].numMessages));
    NMEA0183Handlers[iHandler].Handler(NMEA0183Msg); 
  }
#ifdef WITMOTION
  else {
    if (!strcmp(NMEA0183Msg.Sender(),"PA")) // ignore RTK "PAIR" for now
      log::toAll("unknown 0183 message: " + String(NMEA0183Msg.Sender()) + " " + String(NMEA0183Msg.MessageCode()));
    //for (int i=0; i < NMEA0183Msg.FieldCount(); i++) {
      //log::toAll(String(NMEA0183Msg.Field(i)));
      //Serial.print(NMEA0183Msg.Field(i));
      //if ( i<NMEA0183Msg.FieldCount()-1 ) Serial.print(" ");
    //}
    //Serial.println();
    num_0183_ok++;
  }
#endif
}

// NMEA0183 message Handler functions

void HandleRMC(const tNMEA0183Msg &NMEA0183Msg) {
  //Serial.println("RMC");
  if (pBD==0) return;
  double variation;
  if (NMEA0183ParseRMC_nc(NMEA0183Msg,pBD->GPSTime,pBD->Latitude,pBD->Longitude,pBD->COG,pBD->SOG,pBD->DaysSince1970,pBD->Variation)) {
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
      //SetN2kPGN129025(N2kMsg, pBD->Latitude, pBD->Longitude);
      //if (!n2kMain->SendMsg(N2kMsg)) num_0183_fail++; else num_0183_ok++;
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
  //log::toAll("RMC lat: " + String(pBD->Latitude,5) + " lon: " + String(pBD->Longitude,5) + " COG: " + String(pBD->COG));
  // if we are tuning, output relevant data to compare e.g. paddle log with GPS speed
  if (tuning) {
    sprintf(prbuf, "STW: %2.2f SOG: %2.2f HDG: %2.2f COG: %2.2f", pBD->STW*MTOKTS, pBD->SOG*MTOKTS, pBD->trueHeading, pBD->COG*RADTODEG);
    log::toAll(prbuf);
  }
}

// We might get a sequence of GSV messages, each with up to 4 satellites
// Keep a running count
static int SatCount=0;
static int totalMSG=0;
int maxSat=0;
bool GSVtoggle = true;

void HandleGSV(const tNMEA0183Msg &NMEA0183Msg) {
  if (!GSVtoggle) return; // turn off GSV if it's too much processing
  //Serial.println("GSV");
  int thisMSG;
  int SatelliteCount;
  if (pBD==0) return;
  // SatelliteCount here refers to the total number of satellites in this sequence of 1-3 messages
  if (NMEA0183ParseGSV_nc(NMEA0183Msg, totalMSG, thisMSG, SatelliteCount, SatInfo[0], SatInfo[1], SatInfo[2], SatInfo[3])) {
  } else if (NMEA0183HandlersDebugStream!=0) { NMEA0183HandlersDebugStream->println("Failed to parse GSV"); }
  //log::toAll("GSV sat count: " + String(SatelliteCount) + " msg " + String(thisMSG) + "/" + String(totalMSG));
  if (thisMSG == 1) { // start counting again
    SatCount = 0;
  }
  for (int i=0; i<4; i++) {
    //log::toAll(String(SatInfo[i].SVID) + " el: " + String(SatInfo[i].Elevation) + " az: " + String(SatInfo[i].Azimuth) + " SNR: " + String(SatInfo[i].SNR));
    if (SatInfo[i].SVID > maxSat) maxSat = SatInfo[i].SVID; // IDK what the hell I'm doing with this
    if (SatInfo[i].SVID > MAXSAT) {
      log::toAll("saw higher satellite!!!");
    } else {
      GSVseen[SatInfo[i].SVID] = SatInfo[i];
    }
    SatCount++;
    if (SatCount == SatelliteCount) break;
  }
  if (NMEA0183HandlersDebugStream!=0) {
      NMEA0183HandlersDebugStream->print("totalMSG="); NMEA0183HandlersDebugStream->println(totalMSG);
      NMEA0183HandlersDebugStream->print("thisMSG"); NMEA0183HandlersDebugStream->println(thisMSG);
      NMEA0183HandlersDebugStream->print("SatCount="); NMEA0183HandlersDebugStream->println(pBD->SatelliteCount);
      NMEA0183HandlersDebugStream->print("Sat1"); NMEA0183HandlersDebugStream->println(SatInfo[0].SVID);
  }
}

void HandleGSA(const tNMEA0183Msg &NMEA0183Msg) {
  //Serial.println("GSA");
  return;
}

void HandleGGA(const tNMEA0183Msg &NMEA0183Msg) {
  Serial.println("GGA");
  int SatelliteCount;
  if (pBD==0) return;
  if (NMEA0183ParseGGA_nc(NMEA0183Msg,pBD->GPSTime,pBD->Latitude,pBD->Longitude,
                   pBD->GPSQualityIndicator,SatelliteCount,pBD->HDOP,pBD->Altitude,pBD->GeoidalSeparation,
                   pBD->DGPSAge,pBD->DGPSReferenceStationID)) {
    } else if (NMEA0183HandlersDebugStream!=0) { NMEA0183HandlersDebugStream->println("Failed to parse GGA"); }
  if (abs(SatelliteCount-pBD->SatelliteCount) > 1) {
    //log::toAll("GGA sat count changed old: " + String(pBD->SatelliteCount) + " new: " + String(SatelliteCount));
    pBD->SatelliteCount = SatelliteCount;
  }
  if (n2kMain!=0) {
      tN2kMsg N2kMsg;
      SetN2kGNSS(N2kMsg,1,pBD->DaysSince1970,pBD->GPSTime,pBD->Latitude,pBD->Longitude,pBD->Altitude,
                N2kGNSSt_GPS,GNSMethofNMEA0183ToN2k(pBD->GPSQualityIndicator),pBD->SatelliteCount,pBD->HDOP,0,
                pBD->GeoidalSeparation,1,N2kGNSSt_GPS,pBD->DGPSReferenceStationID,pBD->DGPSAge
                );
      n2kMain->SendMsg(N2kMsg); 
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
    log::toAll("GGA lat: " + String(pBD->Latitude,5) + " lon: " + String(pBD->Longitude,5) + " qual: " + String(pBD->GPSQualityIndicator) + " sat: " + String(pBD->SatelliteCount));
}

void HandleVTG(const tNMEA0183Msg &NMEA0183Msg) {
 double MagneticCOG;
  if (pBD==0) return;
  if (NMEA0183ParseVTG_nc(NMEA0183Msg,pBD->COG,MagneticCOG,pBD->SOG)) {
    //pBD->Variation=pBD->COG-MagneticCOG; // Save variation for Magnetic heading
  } else if (NMEA0183HandlersDebugStream!=0) { NMEA0183HandlersDebugStream->println("Failed to parse VTG"); }
  if (n2kMain!=0) { 
    tN2kMsg N2kMsg;
    SetN2kCOGSOGRapid(N2kMsg,1,N2khr_true,pBD->COG,pBD->SOG);
    n2kMain->SendMsg(N2kMsg);
//      SetN2kBoatSpeed(N2kMsg,1,SOG);
//      n2kMain.SendMsg(N2kMsg);
  }
  if (NMEA0183HandlersDebugStream!=0) {
    NMEA0183HandlersDebugStream->print("True heading="); NMEA0183HandlersDebugStream->println(pBD->trueHeading);
  }
}

void HandleGLL(const tNMEA0183Msg &NMEA0183Msg) {
  //Serial.println("GLL");
  return;
}

double NMEA0183GetDouble(const char *data);

#ifdef WITMOTION
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
    log::toAll(logMsg);
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
    //log::toAll("PQTMTAR q: " + String(Quality) + " l: " + String(Length) + " p: " + String(Pitch) + " r: " + String(Roll) + " h: " + String(Roll) + " sv: " + String(UsedSV));
  }
}
#endif

// these messages are from the UM982 TBD: add #define so I can switch back to Witmotion if necessary
//*****************************************************************************
// $GNHPR,074615.00,320.9610,-66.1712,000.0000,4,47,0.00,0999*45
//        UTC      ,HEADING ,PITCH   ,ROLL    ,QUAL,Sat#,Age,stnID,CKSUM
bool NMEA0183ParseHPR(const tNMEA0183Msg &NMEA0183Msg, double &UTC, double &Heading, double &Pitch, double &Roll, int &QF, int &SatNo, double &Age, int &Station) {
  bool result=( NMEA0183Msg.FieldCount()>=7 );
#if 0
  for (int i=0; i < NMEA0183Msg.FieldCount(); i++) {
    Serial.printf("%d: %s ",i, NMEA0183Msg.Field(i));
    //if (i<NMEA0183Msg.FieldCount()-1 ) Serial.print(" ");
  }
  Serial.println(); 
#endif
  if (result) { 
    UTC=NMEA0183GetDouble(NMEA0183Msg.Field(0));
    Heading=NMEA0183GetDouble(NMEA0183Msg.Field(1));
    Pitch=NMEA0183GetDouble(NMEA0183Msg.Field(2));
    Roll=NMEA0183GetDouble(NMEA0183Msg.Field(3));
    QF=atoi(NMEA0183Msg.Field(4));
    SatNo=atoi(NMEA0183Msg.Field(5));
    Age=NMEA0183GetDouble(NMEA0183Msg.Field(6));
    Station=atoi(NMEA0183Msg.Field(7));
  }
  return result;
}

/*
Quality:
0 = Fix invalid
1 = Single point positioning 
2 = Differential GPS
4 = RTK fix
5 = RTK float
6 = Dead reckoning mode
7 = Manual input mode (fixed value) 
8 = Extra wide-lane
9 = SBAS
*/

void HandleHPR(const tNMEA0183Msg &NMEA0183Msg) {
  double UTC, Heading, Pitch, Roll, Age;
  int QF, SatNo, Station;
  if (pBD==0) return;
  if (NMEA0183ParseHPR(NMEA0183Msg, UTC, Heading, Pitch, Roll, QF, SatNo, Age, Station)) {
    if (Heading > 0.01 && Pitch > 0.01 && Roll > 0.01) {
      // sensor is currently sending 0 for HPR so only set heading if all are >0
      pBD->trueHeading=fmod(Heading+rtkOrientation, 359.9);
      pRTK->GPStime = UTC;
      pRTK->RTKqual = QF; // quality of fix
      pRTK->heading = pBD->trueHeading;
      pRTK->pitch = Pitch;
      pRTK->roll = Roll;
      pRTK->usedSV = SatNo;
    } else {
      // use magnetic heading
      pBD->trueHeading=fmod(pBD->magHeading+pBD->Variation,359.9);
      if (rtkDebug) {
        //log::toAll("setting heading from compass " + String(pBD->magHeading));
      }
    }
  } else if (NMEA0183HandlersDebugStream!=0) { NMEA0183HandlersDebugStream->println("Failed to parse HPR"); }
  if (n2kMain!=0) { 
    tN2kMsg N2kMsg;
    // Vessel Heading (deviation should always be 0 since it's not a magnetic compass)
    // heading arrives in DEGREES convert to radians for n2k
    if (rtkDebug) {
      //log::toAll("sending n2k true heading " + String(pBD->trueHeading) + " variation " + String(pBD->Variation));
    }
    SetN2kPGN127250(N2kMsg, 1, pBD->trueHeading*DEGTORAD, 0, pBD->Variation*DEGTORAD, N2khr_true);
    n2kMain->SendMsg(N2kMsg);
    //log::toAll("sending 127250");
    // Attitude
    //setN2kPGN127257();
  }
  if (NMEA0183HandlersDebugStream!=0) {
    NMEA0183HandlersDebugStream->print("True heading="); NMEA0183HandlersDebugStream->println(pBD->trueHeading);
    NMEA0183HandlersDebugStream->print("Pitch="); NMEA0183HandlersDebugStream->println(Pitch);
    NMEA0183HandlersDebugStream->print("Roll="); NMEA0183HandlersDebugStream->println(Roll);
    NMEA0183HandlersDebugStream->print("Quality Fix="); NMEA0183HandlersDebugStream->println(QF);
    NMEA0183HandlersDebugStream->print("SatNo="); NMEA0183HandlersDebugStream->println(SatNo);
    NMEA0183HandlersDebugStream->print("Age="); NMEA0183HandlersDebugStream->println(Age);
    NMEA0183HandlersDebugStream->print("Station="); NMEA0183HandlersDebugStream->println(Station);
  }
}
#endif
