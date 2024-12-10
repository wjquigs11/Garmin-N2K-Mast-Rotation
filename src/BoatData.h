#ifndef _BoatData_H_
#define _BoatData_H_

#define MTOKTS 1.943844

struct tBoatData {
  unsigned long DaysSince1970;   // Days since 1970-01-01
  
  double TrueHeading,SOG,COG,Variation,
         STW, // meters/sec
         TWS, TWA, // meters/sec
         maxTWS,
         GPSTime,// Secs since midnight,
         Latitude, Longitude, Altitude, HDOP, GeoidalSeparation, DGPSAge;
  int GPSQualityIndicator, SatelliteCount, DGPSReferenceStationID;
  bool MOBActivated;

public:
  tBoatData() {
    TrueHeading=0;
    SOG=0;
    COG=0; 
    Variation=0.0;
    STW=0.0;
    TWS=0.0; TWA=0.0;
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

#endif // _BoatData_H_

