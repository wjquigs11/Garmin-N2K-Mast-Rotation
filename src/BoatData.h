#ifndef _BoatData_H_
#define _BoatData_H_

struct tBoatData {
  unsigned long DaysSince1970;   // Days since 1970-01-01
  
  float TrueHeading,COG,Variation,
         STW, SOG,// meters/sec
         GPSTime,// Secs since midnight,
         Latitude, Longitude, Altitude, HDOP, GeoidalSeparation, DGPSAge;
  int GPSQualityIndicator, SatelliteCount, DGPSReferenceStationID;
  bool MOBActivated;
  float magHeading;

public:
  tBoatData() {
    TrueHeading=0;
    SOG=0;
    COG=0; 
    Variation=0.0;
    STW=0.0,
    GPSTime=0;
    Altitude=0;
    HDOP=100000;
    DGPSAge=100000;
    DaysSince1970=0; 
    MOBActivated=false; 
    SatelliteCount=0; 
    DGPSReferenceStationID=0;
    magHeading=0.0;
  };
};

extern tBoatData BoatData;

#endif // _BoatData_H_

