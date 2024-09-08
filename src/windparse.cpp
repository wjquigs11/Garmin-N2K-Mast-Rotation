// parse wind speed, correct for mast rotation
//#include <Arduino.h>
#include <ActisenseReader.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <N2kMessages.h>
#include <NMEA2000_esp32.h>
#include "BoatData.h"
#include <ReactESP.h>
#include <Wire.h>
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>
#include <movingAvg.h>
#include "elapsedMillis.h"
#include <Arduino.h>
#include <N2kMessages.h>
#include <Adafruit_ADS1X15.h>
#include <WiFi.h>
#include "SPIFFS.h"
#include <Arduino_JSON.h>
#include <ESPmDNS.h>
//#include "mcp2515.h"
//#include "can.h"
#include <ESPAsyncWebServer.h>
//#include <ElegantOTA.h>
#include <WebSerial.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include "esp_system.h"
#include "esp32-hal-log.h"

#include "compass.h"
#include "windparse.h"

#ifdef PICAN
#include <N2kMsg.h>
#include <NMEA2000.h>
#include <SPI.h>
#include <mcp_can.h>
#include <NMEA2000_mcp.h>
#ifdef NMEA0183
#include <NMEA0183.h>
#include <NMEA0183Msg.h>
#include <NMEA0183Messages.h>
//#include "NMEA0183Handlers.h"
extern tNMEA0183 NMEA0183_3;
tNMEA0183Msg NMEA0183Msg;
#define NMEA0183serial Serial1
#endif
#elif RS485CAN
#include "mcp2515_can.h"
#endif

float mastRotate, rotateout;
// analog values from rotation sensor
int PotValue=0;
int PotLo=9999;
int PotHi=0;
extern int portRange, stbdRange; // NB BOTH are positive (from web calibration)
extern bool compassOnToggle, honeywellOnToggle;
extern float mastCompassDeg, boatCompassDeg, mastDelta;
extern movingAvg mastCompDelta;
extern int mastOrientation;   // delta between mast compass and boat compass
extern int sensOrientation;
extern int boatOrientation;
float getCompass(int correction);
extern tBoatData BoatData;
void logToAll(String s);

// Initialize static variables for RotationSensor Class
int RotationSensor::newValue{0};
int RotationSensor::oldValue{0};

// not sure if I need these any more since I'm doing it all in one function
double WindSensor::windSpeedKnots{0.0};
double WindSensor::windAngleDegrees{0.0};
double WindSensor::windSpeedMeters{0.0};
double WindSensor::windAngleRadians{0.0};
tN2kWindReference wRef;
double SpeedThruWater; // meters/sec
double TWS; // meters/sec
int TWA; // radians

movingAvg honeywellSensor(10);
Adafruit_ADS1015 ads;
int adsInit;

extern tNMEA2000 *n2kMain;
extern int num_wind_messages;

extern JSONVar readings;

char prbuf[PRBUF];

void calcTrueWind();

// returns degrees, and corresponds to the current value of the Honeywell sensor
float readAnalogRotationValue() {      
#if defined(SH_ESP32)
  PotValue = analogRead(POT_PIN);
#else
  if (adsInit)
    PotValue = ads.readADC_SingleEnded(0);
#endif
  //int AltValue = adc1_get_raw(ADC1_CHANNEL_5);
  if (!PotValue)
    return 0;
  // determine range of A2D values; this might be different on your boat
  // after calibration, a value lower than PotLo or higher than PotHi should cause an error
  if (PotValue < PotLo && PotValue > 0) { 
    PotLo = PotValue;
    readings["PotLo"] = String(PotLo); // for calibration
  } else if (PotValue > PotHi) {
    PotHi = PotValue;
    readings["PotHi"] = String(PotHi); // for calibration
  }
  #ifdef DEBUG
  sprintf(prbuf, " pot(l/v/h): %d/%d/%d ", PotLo, PotValue, PotHi);
  //Serial.print(prbuf);
  #endif
  // the moving average variable is only initialized if ADC is present
  int newValue = honeywellSensor.reading(PotValue);    // calculate the moving average
  int oldValue = RotationSensor::oldValue;
  
  if (newValue < highset) {  // writes value to oldsensor if below highset threshold
    oldValue = newValue;
  }
  // Update values for new and old values (for the next loop iteration)
  RotationSensor::newValue = newValue;
  RotationSensor::oldValue = oldValue;

  // map 10 bit number to degrees of rotation
  //sprintf(prbuf,"map: %d %d %d %d %d = %0.2f\n", oldValue, lowset, highset, -portRange, stbdRange, mastAngle[0]);
  //logToAll(prbuf);
  mastAngle[0] = map(oldValue, lowset, highset, -portRange, stbdRange)+sensOrientation;    
  #ifdef DEBUG2
  sprintf(prbuf, "%d %d %d %d", oldValue, lowset, highset, mastRot);
  //Serial.println(prbuf);
  #endif
  return mastAngle[0]; 
}

int compassDifference(int angle1, int angle2) {
    int diff = (angle1 - angle2 + 360) % 360;
    //Serial.print("compdiff: "); Serial.println(diff);
    return (diff > 180) ? (diff - 360) : diff;
}

float readCompassDelta() {
  if (compassReady) {
    mastDelta = compassDifference(boatCompassDeg, mastCompassDeg);
    //logToAll("mastDelta: " + String(mastDelta));
    mastAngle[1] = mastDelta;
    mastCompDelta.reading((int)(mastDelta*100)); // moving average
    return mastDelta;
  }
  //Serial.println("compass not ready");
  return -1;
}

tN2kMsg correctN2kMsg;

byte calculateNMEAChecksum(const char* sentence) {
  byte checksum = 0;
  // Start after the '$' and continue until '*' or end of string
  for (int i = 1; sentence[i] != '\0' && sentence[i] != '*'; i++) {
    checksum ^= sentence[i];
  }
  return checksum;
}

#ifdef NMEA0183
char n0183buf[64],n0183cksumbuf[64];
#endif

void WindSpeed() {
  ////Serial.println("windspeed");
  num_wind_messages++;
  WindSensor::windSpeedKnots = WindSensor::windSpeedMeters * 1.943844; // convert m/s to kts
  WindSensor::windAngleDegrees = WindSensor::windAngleRadians * RADTODEG;
  ////Serial.printf("sensor angle %0.2f\n", WindSensor::windAngleDegrees);
  if (wRef != N2kWind_Apparent) { // N2kWind_Apparent
    //Serial.printf("got wind PGN not apparent! %d\n", wRef);
    return;
  }
  // read rotation value and correct
  // either read from Honeywell sensor, or from external compass, or both
  // if external compass, reading will be updated in espnow.cpp
  mastRotate = 0.0;
  if (honeywellOnToggle) {
    mastRotate = readAnalogRotationValue();
    ////Serial.printf("honeywell mastrotate = %d\n", (int)mastRotate);
  }
  if (compassOnToggle) {
    // only use compass if Honeywell not enabled
    if (!honeywellOnToggle)
      mastRotate = readCompassDelta();
  }
  #ifdef DEBUG
  //Serial.print(" mastRotate: ");
  //Serial.print(mastRotate);
  #endif
  float anglesum = WindSensor::windAngleDegrees + mastRotate;    // sensor AFT of mast so subtract rotation
  // ensure sum is 0-359; rotateout holds the corrected AWA
  if (anglesum<0) {                             
    rotateout = anglesum + 360;
  }              
  else if (anglesum>359) {   
    rotateout = anglesum - 360;               
  }
  else {
    rotateout = anglesum;               
  }
  #ifdef DEBUG
  logToAll("rotate now " + String(PotValue) + " low " + String(PotLo) + " hi " + String(PotHi));
  logToAll("mastrotate " + String(mastRotate) + " anglesum " + String(anglesum) + " rotateout " + String(rotateout));
  #endif
  // send corrected wind on main bus
  // note we are sending the original speed reading in m/s
  // and the AWA converted from rads to degrees, corrected, and converted back to rads
  SetN2kPGN130306(correctN2kMsg, 0xFF, WindSensor::windSpeedMeters, rotateout*DEGTORAD, N2kWind_Apparent); 
  if (n2kMain->SendMsg(correctN2kMsg)) {
    ////Serial.printf("sent n2k wind %0.2f", rotateout);
  } else {
    Serial.println("Failed to send wind");  
  }
#ifdef NMEA0183
  // send on NMEA0183 serial (to Tiller Pilot)
  ////Serial.printf("0183 vars: %x %f %d %f %x\n", &NMEA0183Msg, rotateout, NMEA0183Wind_Apparent, WindSensor::windSpeedMeters,NMEA0183_3);
  float bowAngle;
  char direction;
  if (rotateout > 180) {
    bowAngle = 360 - rotateout;
    direction = 'L'; // Left
  } else {
    bowAngle = rotateout;
    direction = 'R'; // Right
  }
  sprintf(n0183buf,"$IIVWR,%2d,%c,%2.1f,N,%2.1f,M,%2.1f,K*",(int)rotateout, direction, WindSensor::windSpeedKnots, WindSensor::windSpeedMeters, WindSensor::windSpeedMeters/3.6);
  int cksum = calculateNMEAChecksum(n0183buf);
  sprintf(n0183cksumbuf,"%s%02X", n0183buf, cksum);
  //Serial.println(buf2);
  NMEA0183serial.println(n0183cksumbuf);
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

#ifdef PICAN
void ParseWindN2K(const tN2kMsg &N2kMsg) {
  unsigned char SID;  
  if (ParseN2kPGN130306(N2kMsg, SID, WindSensor::windSpeedMeters, WindSensor::windAngleRadians, wRef)) {
    ////Serial.printf("N2K parsed wind SID %d Speed %0.2f Angle %0.2f ref %d\n", SID, WindSensor::windSpeedMeters, WindSensor::windAngleRadians, wRef);
  } else Serial.printf("no parse\n");
  WindSpeed();
}

// parse compass reading on wind bus
// TBD: decide if we're on wind bus or main bus, because heading from wind bus is mast compass 
// and heading from main bus is external compass
// for now we're just assuming we're on the wind bus and it's the mast compass
// also reusing rudder angle (AGAIN) to transmit mast compass heading on N2K (NO)
void ParseCompassN2K(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  double heading;
  double deviation;
  double variation;
  tN2kHeadingReference headingRef;
  //logToAll("parsecompassn2k");
  if (ParseN2kPGN127250(N2kMsg, SID, heading, deviation, variation, headingRef)) {
    // TBD get "reference" to confirm it's N2khr_Unavailable
    mastCompassDeg = heading * RADTODEG;
    readCompassDelta();
    // NOTE we do NOT transmit boat heading on N2K here; only from reaction in wind-bus.cpp, to avoid flooding bus
//#define XMITRUDDER
#ifdef XMITRUDDER // send rudder angle (as rudder #1)
    //SetN2kPGN127245(correctN2kMsg, heading, 1, N2kRDO_NoDirectionOrder, 0);
    SetN2kPGN127250(correctN2kMsg, 0xFF, (double)heading, N2kDoubleNA, N2kDoubleNA, N2khr_Unavailable);
    n2kMain->SendMsg(correctN2kMsg);
    logToAll("sent rudder " + String(heading) + " rad " + String(heading*DEGTORAD));
#endif
  } else {
    // no boat compass, use external heading info
    //if (compassOnToggle) {
    // got bearing; calculate mast heading
    // TBD: this isn't correct
    //tN2kMsg correctN2kMsg;
    //int mastRotate = convertMagHeading(N2kMsg);
    //if (mastRotate > -1) {
      // for now (until you dive into SensESP), send rotation angle as "rate of turn"
      //SetN2kPGN127251(correctN2kMsg,0xff,(mastRotate+50)*(M_PI/180));
      //n2kMain->SendMsg(correctN2kMsg);
      //}
    //} 
    // !compassOnToggle (no internal compass), so set boat heading here
    // TBD: check if we're getting true or magnetic
    //BoatData.TrueHeading = heading;
    //BoatData.Variation = variation;
  } // else
}
#endif

// process boat speed to update STW for true wind calc
void BoatSpeed(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  double SpeedWaterMeters;
  double SpeedGroundMeters; // don't really care about ground speed here; TWS and TWA should refer to boat speed thru water
  tN2kSpeedWaterReferenceType SWRT;
  if (ParseN2kPGN128259(N2kMsg, SID, SpeedWaterMeters, SpeedGroundMeters, SWRT)) {
    SpeedThruWater = SpeedWaterMeters; // KEEP METERS * 1.943844; // convert m/s to kts
    BoatData.STW = SpeedThruWater;
  }
}

void calcTrueWind() {
  // note using meters/sec and radians
  ////Serial.printf("WS(m): %2.2f WA(r): %2.2f cos: %2.2f\n", WindSensor::windSpeedMeters, WindSensor::windAngleRadians, cos(WindSensor::windAngleRadians));
  double AWS = WindSensor::windSpeedMeters;
  double AWA = WindSensor::windAngleRadians;
  double STW = SpeedThruWater;
  TWS = sqrt(STW*STW + AWS*AWS - 2 * STW * AWS * cos(WindSensor::windAngleRadians));
  // Calculate the component of AWS in the direction of the vessel's motion
  double AWS_parallel = AWS * sin(AWA);
  // Calculate the angle between the true wind direction and the vessel's heading
  TWA = acos((STW * cos(AWA) - AWS_parallel) / TWS);
//#define DEBUG
#ifdef DEBUG
  //Serial.printf("STW(k): %2.2f TWS(k): %2.2f TWA(d): %2.2f\n", STW*1.943844, TWS*1.943844, TWA*(180/M_PI));
#endif
}




