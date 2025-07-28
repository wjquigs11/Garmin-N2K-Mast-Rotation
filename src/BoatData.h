#ifndef _BoatData_H_
#define _BoatData_H_

#define MTOKTS 1.943844

struct tBoatData {
  unsigned long DaysSince1970;   // Days since 1970-01-01
  
  // degrees not radians
  // remember to convert if you transmit in n2k
  double trueHeading,SOG,COG,Variation,
         magHeading,
         STW, // meters/sec
         TWS, TWA, // meters/sec and DEGREES NOT RADIANS
         TWD, // relative to north
         VMG, // meters/sec (velocity made good to wind)
         maxTWS,
         GPSTime,// Secs since midnight,
         Latitude, Longitude, Altitude, HDOP, GeoidalSeparation, DGPSAge;
  int GPSQualityIndicator, SatelliteCount, DGPSReferenceStationID;
  bool MOBActivated;

public:
  tBoatData() {
    trueHeading=-1.0; // init to -1 in case we have no sources for heading
    SOG=0.0;
    COG=0.0; 
    Variation=0.0;
    magHeading=0.0;
    STW=0.0;
    TWS=0.0; TWA=0.0;
    TWD=0.0;
    VMG=0.0;
    maxTWS=0.0;
    GPSTime=0;
    Altitude=0;
    HDOP=100000;
    DGPSAge=100000;
    DaysSince1970=0; 
    MOBActivated=false; 
    SatelliteCount=0; 
    DGPSReferenceStationID=0;
  };
};

struct tRTKstats {
  double antennaAstat, antennaBstat, baseLen, GPStime, pitch, roll, heading, pAcc, rAcc, hAcc, usedSV;
  int RTKqual;

public:
  tRTKstats() {
    antennaAstat=0;
    antennaBstat=0;
    baseLen=0;
    GPStime=0;
    RTKqual=0;
    pitch=0;
    roll=0;
    heading=0;
    pAcc=0;
    rAcc=0;
    hAcc=0;
    usedSV=0;
  };
};

struct tEnvStats {
  float temp, pressure, humidity;
public:
  tEnvStats() {
    temp=0;
    pressure=0;
    humidity=0;
  };
};

extern tBoatData *pBD;
extern tBoatData BoatData;
extern tRTKstats *pRTK;
extern tRTKstats RTKdata;
extern tEnvStats *pENV;
extern tEnvStats ENVdata;

// redefine value in NMEA0183.h
// UM982 sends NMEA0183 messages longer than 81 chars
#define MAX_NMEA0183_MSG_BUF_LEN 101

#endif // _BoatData_H_

