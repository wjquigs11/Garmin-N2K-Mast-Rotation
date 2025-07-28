// parse wind speed, correct for mast rotation

#include "compass.h"
#include "windparse.h"
#include "BoatData.h"

// object-oriented classes
#include "BNO085Compass.h"
#include "logto.h"

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
#endif

#ifdef RS485CAN
#include "mcp2515_can.h"
#endif

float mastRotate, rotateout;
// analog values from rotation sensor
int PotValue=0;
int PotLo=9999;
int PotHi=0;
bool logPot = false;
extern int portRange, stbdRange; // NB BOTH are positive (from web calibration)
extern bool honeywellOnToggle;
extern float mastCompassDeg, mastDelta;
extern bool passThru;
#ifdef BNO_GRV
extern movingAvg mastCompDelta;
#endif
/*
extern int mastOrientation;   // delta between mast compass and boat compass
extern int sensOrientation;
extern int boatOrientation;
*/
float getCompass(int correction);
//void log::toAll(String s);

// Initialize static variables for RotationSensor Class
int RotationSensor::newValue{0};
int RotationSensor::oldValue{0};

// not sure if I need these any more since I'm doing it all in one function
double WindSensor::windSpeedKnots{0.0};
double WindSensor::windAngleDegrees{0.0};
double WindSensor::windSpeedMeters{0.0};
double WindSensor::windAngleRadians{0.0};
tN2kWindReference wRef;
tN2kHeadingReference hRef;
//double SpeedThruWater; // meters/sec
//double TWS; // meters/sec
//int TWA; // radians

#ifdef HONEY
movingAvg honeywellSensor(5);
Adafruit_ADS1015 ads;
int adsInit;
#endif

extern tNMEA2000 *n2kMain;
extern int num_wind_messages;
extern int num_wind_other;  // number of non-wind PGNs on wind bus
extern int num_wind_fail; // number of failed wind messages
extern int num_wind_ok; // number of successful wind messages
extern int num_wind_other_fail; // number of failed non-wind PGNs on wind bus
extern int num_wind_other_ok; // number of successful non-wind PGNs on wind bus
// an array of PGNs that we've observed on the wind bus that are *not* 127250 or 130306
unsigned long otherPGN[MAXPGN];
int otherPGNindex = 0;

extern elapsedMillis time_since_last_mastcomp_rx;

extern JSONVar readings;

char prbuf[PRBUF];

void calcTrueWind();
void windCounter();
void mastCompCounter();
//#define DEBUG
#ifdef HONEY
// returns degrees, and corresponds to the current value of the Honeywell sensor
float readAnalogRotationValue() {      
#if defined(SH_ESP32)
  PotValue = analogRead(POT_PIN);
#else
#ifdef INA219
  float busvoltage = ina219.getBusVoltage_V();
  Serial.printf("busvoltage: %f\n", busvoltage);
  mastAngle[0] = map(busvoltage, lowset, highset, -portRange, stbdRange)+sensOrientation;
  return mastAngle[0];
#else
  if (adsInit)
    PotValue = ads.readADC_SingleEnded(0);
#endif // INA219
#endif // SH_ESP32
  //int AltValue = adc1_get_raw(ADC1_CHANNEL_5);
  if (!PotValue)
    return 0;
  // determine range of A2D values; this might be different on your boat
  // after calibration, a value lower than PotLo or higher than PotHi should cause an error
  if (PotValue < PotLo && PotValue > 0) { 
    PotLo = PotValue;
    readings["PotLo"] = String(PotLo); // for calibration
    if (PotLo < lowset)
      log::toAll("WARNING! PotValue lower than lowset " + String(PotLo));
  } else if (PotValue > PotHi) {
    PotHi = PotValue;
    readings["PotHi"] = String(PotHi); // for calibration
    if (PotHi > highset) 
      log::toAll("WARNING! PotValue greater than highset value:" + String(PotHi) + " highset:" + String(highset));
  }
  if (logPot) {
    sprintf(prbuf, " pot low:%d (lowset:%d)/current: %d/high: %d (highset:%d)", PotLo, lowset, PotValue, PotHi, highset);
    log::toAll(prbuf);
  }
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
  mastAngle[0] = map(oldValue, lowset, highset, -portRange, stbdRange)+sensOrientation;
  //sprintf(prbuf,"map: %d %d %d %d %d = %ld\n", oldValue, lowset, highset, -portRange, stbdRange, mastAngle[0]);
  //log::toAll(prbuf);
  return -mastAngle[0]; 
}
#endif

int compassDifference(int angle1, int angle2) {
//    int diff = (angle1 - angle2 + 360) % 360;
//    return (diff > 180) ? (diff - 360) : diff;
  int diff = ((angle2 - angle1 + 180) % 360) - 180;
  return diff;
}

float readCompassDelta() {
  if (imuReady) {
    float mastDelta = compassDifference(compass.boatIMU, mastCompassDeg+mastOrientation);
    //log::toAll("readCompassDelta m: " + String(mastCompassDeg+mastOrientation) + " b: " + String(compass.boatIMU) + " delta: " + String(mastDelta));
    mastAngle[1] = mastDelta;
    //reading((int)(mastDelta*100)); // moving average
    return mastDelta;
  }
  //Serial.println("compass not ready");
  return -1;
}

tN2kMsg correctN2kMsg;

#ifdef NMEA0183
byte calculateNMEAChecksum(const char* sentence) {
  byte checksum = 0;
  // Start after the '$' and continue until '*' or end of string
  for (int i = 1; sentence[i] != '\0' && sentence[i] != '*'; i++) {
    checksum ^= sentence[i];
  }
  return checksum;
}

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
  mastRotate = 0.0;
#ifdef HONEY
  if (honeywellOnToggle) {
    mastRotate = readAnalogRotationValue();
#ifdef BNO_GRV
    // the only time this will trigger is if I get a Wind packet at the same moment as the mast is centered.
    // might be better to poll the rotation sensor when I get a packet from the mast compass on the wind bus
    // I could also "correct" mast compass based on Honeywell sensor, although that would invalidate the comparison between the two
    if (abs(mastRotate) < 1) {
      // mast is centered; reset delta between mast IMU and boat IMU
      log::toAll("WindSpeed: got center trigger, prev orientation mast: " + String(mastCompassDeg) + " boat: " + String(compass.boatIMU) + " delta: " + String(mastDelta));
      mastOrientation = 0;
      mastOrientation = mastDelta = readCompassDelta();
      log::toAll("new orientation: " + String(mastDelta));
    }
#endif
#ifdef MASTIMU
    //mastRotate = mastOrientation; // read from Rudder PGN on Wind bus from mast IMU
    mastRotate = readCompassDelta();
#endif
#ifdef XMITRUDDER
    // shift from -portRange..stbdRange to 0..x
    SetN2kPGN127245(correctN2kMsg, (mastRotate+portRange)*DEGTORAD, 0, N2kRDO_NoDirectionOrder, 0);
    n2kMain->SendMsg(correctN2kMsg);
#endif
  }
#endif
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
  #if 0
  log::toAll("rotate now " + String(PotValue) + " low " + String(PotLo) + " hi " + String(PotHi));
  log::toAll("mastrotate " + String(mastRotate) + " anglesum " + String(anglesum) + " rotateout " + String(rotateout));
  #endif
  // send corrected wind on main bus
  // note we are sending the original speed reading in m/s
  // and the AWA converted from rads to degrees, corrected, and converted back to rads
  SetN2kPGN130306(correctN2kMsg, 0xFF, WindSensor::windSpeedMeters, rotateout*DEGTORAD, N2kWind_Apparent); 
  if (n2kMain->SendMsg(correctN2kMsg)) {
    ////Serial.printf("sent n2k wind %0.2f", rotateout);
  } else {
    //Serial.println("Failed to send wind");  
    num_wind_other_fail++;
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
  //calcTrueWind();
#ifdef XMITTRUE
  // calculate TWS/TWA from boat speed and send another wind PGN
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
void ParseCompassN2K(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  double heading;
  double deviation;
  double variation;
  tN2kHeadingReference headingRef;
  if (ParseN2kPGN127250(N2kMsg, SID, heading, deviation, variation, headingRef)) {
    //log::toAll("wind heading ref " + String(headingRef));
    mastCompassDeg = heading * RADTODEG + BoatData.Variation;
    mastRotate = compassDifference(mastCompassDeg, BoatData.trueHeading);
    // NOTE we do NOT transmit boat heading on N2K here; only from reaction in wind-bus.cpp, to avoid flooding bus
  } else {
    // no boat compass, use external heading info
    //if (compass.OnToggle) {
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
    // !compass.OnToggle (no internal compass), so set boat heading here
    // TBD: check if we're getting true or magnetic
    //BoatData.trueHeading = heading;
    //BoatData.Variation = variation;
  } // else
}
#endif

//#define DEBUG
#ifdef RS485CAN
extern mcp2515_can n2kWind;
//extern tN2kMsg correctN2kMsg;
int windCANsrc = -1;
//extern mcp2515_can n2kWind;
byte cdata[MAX_DATA_SIZE] = {0};
//#define DEBUG
// parse a packet manually from CAN bus data (not using Timo library)
// set globals for wind speed/angle or heading for processing by WindSpeed() or Heading()
void parseWindCAN() {
  //Serial.print("parsewindcan ");
  uint8_t len;
  int PGN;
  unsigned char SID;  
  if (n2kWind.checkReceive() != CAN_MSGAVAIL) {
    //Serial.println("no more on wind bus");
    return;
  }
  windCounter();
  n2kWind.readMsgBuf(&len, cdata);
  unsigned long ID = n2kWind.getCanId();
  PGN = ((ID & 0x1FFFFFFF)>>8) & 0x3FFFF; // mask 00000000000000111111111111111111
  uint8_t SRC = ID & 0xFF;
  SID = cdata[0];
#ifdef DEBUG
  Serial.printf("CAN PGN %d SRC %d SID %d len %d\n", PGN, SRC, SID, len);
  for (int i=0; i<len; i++)
    Serial.printf("%x:%d ",cdata[i],cdata[i]);
  Serial.println();
#endif
  if (!passThrough)
    switch (PGN) {
      case 130306: { 
        if (windCANsrc == -1) { // this is our first wind packet, track the source
          windCANsrc = SRC;
          log::toAll("got first wind from CAN " + String(windCANsrc));
        }
        WindSensor::windSpeedMeters = ((cdata[2] << 8) | cdata[1]) / 100.0;
        WindSensor::windAngleRadians = ((cdata[4] << 8) | cdata[3]) / 10000.0;
        wRef = (tN2kWindReference)(cdata[5]&0x07);
  #ifdef DEBUGCAN
        Serial.printf("CAN parsed wind SID %d Speed %0.2f Angle %0.4f (%0.4f) ref %d (%x)\n", SID, WindSensor::windSpeedMeters, WindSensor::windAngleRadians, WindSensor::windAngleRadians*(180/M_PI), wRef, cdata[5]);
          for (int i=0; i<8; i++)
            Serial.printf("%02X ", cdata[i]);
          Serial.println();
  #endif
        if (time_since_last_mastcomp_rx > 5000) {
          // if mast messages timeout, send unchanged wind PGN
          SetN2kPGN130306(correctN2kMsg, 0xFF, WindSensor::windSpeedMeters, WindSensor::windAngleRadians, N2kWind_Apparent); 
          if (n2kMain->SendMsg(correctN2kMsg)) {
            //Serial.printf("sent n2k wind %0.2f", rotateout);
          } else {
            //Serial.println("Failed to send wind from parseWindCAN()");  
            num_wind_other_fail++;
          }
        } else 
        WindSpeed();
        //break;
        return; // so as not to pass-through this packet
      } 
      case 127250: {
        mastCompCounter(); // for timeout if it gets disconnected
        float mastCompassRad = ((cdata[2] << 8) | cdata[1]) / 10000.0;
        // deviation 4|3, variation 6|5
        hRef = (tN2kHeadingReference)(cdata[7]&0x03);
        /*
        if (hRef == N2khr_umavailable) { // expected from mast IMU
              mastCompassDeg = mastCompassRad * RADTODEG;
        }
        // hack! if hRef == N2khr_true that means mast IMU is aligned with Hall sensor
        if (hRef == N2khr_true) {
          mastCompassDeg = mastCompassRad * RADTODEG;
          log::toAll("got true heading trigger, setting orientation mast: " + String(mastCompassDeg) + " boat: " + String(compass.boatIMU) + " delta: " + String(mastDelta));
          mastOrientation = 0;
          mastOrientation = mastDelta = readCompassDelta();
        }
        */
        if (hRef == N2khr_magnetic) {
          mastCompassDeg = mastCompassRad * RADTODEG + BoatData.Variation;
          mastRotate = compassDifference(mastCompassDeg, BoatData.trueHeading);
          sprintf(prbuf,"compass heading, mast(t): %2.2f boat(t): %2.2f cdelta: %2.2f",mastCompassDeg, BoatData.trueHeading, mastRotate);
          log::toAll(prbuf);
        }
  #ifdef DEBUG
        Serial.printf("CAN parsed heading SID %d heading %0.4f ref %d (%x)\n", SID, mastCompassDeg, hRef, cdata[5]);
  #endif
        //break;  // don't forward heading
        return;
      }
      case 127245: {
        // Parse rudder data manually from CAN bus
        // Extract rudder position directly from bytes 5-6 (little-endian)
        int16_t rawValue = (cdata[5] << 8) | cdata[4];
        mastOrientation = rawValue * 0.0001 * RADTODEG;
        if (compass.teleplot) Serial.printf(">gyro:%0.2f\n",mastOrientation);
          else log::toAll("mast/rudder angle: " + String(mastOrientation,2));
        //break;
        return;
      }
    } // switch(PGN)
    // switch should return for above PGNs
    // if passThrough = true, we will come here
    //default: { 
      num_wind_other++; 
      bool PGNfound=false;
      for (int i=0; i<otherPGNindex; i++) {
        if (otherPGN[i] == PGN) {
          PGNfound=true;
          //break;
          return;
        }
      }
      if (!PGNfound) {
        log::toAll("new wind PGN " + String(PGN));
        otherPGN[otherPGNindex++] = PGN;
      }
      // pass through everything that's not covered above (wind and heading)
      correctN2kMsg.SetPGN(PGN);
      correctN2kMsg.Priority=2;
      correctN2kMsg.AddByte(SID);
      for (int i=1; i<len; i++) {
        correctN2kMsg.AddByte(cdata[i]);
      }
      if (n2kMain->SendMsg(correctN2kMsg)) {
        num_wind_other_ok++;
        //Serial.printf("forward N2k OK %d\n", PGN);
      } else {
        num_wind_other_fail++;
        if (num_wind_other_fail % 100 == 1)
          log::toAll("Failed to pass through from wind to main PGN: " + String(PGN));  
      }
      //break;
    //}
}
#endif

#if 0
// keep this code around in case I want to use the library by building a message from raw data and parsing it
      correctN2kMsg.SetPGN(127245L);
      correctN2kMsg.Priority=2;
      for (int i=0; i<len; i++)
        correctN2kMsg.Data[i]=cdata[i];
      correctN2kMsg.DataLen = len;
      double RudderPosition;
      unsigned char Instance;
      tN2kRudderDirectionOrder RudderDirectionOrder;
      double AngleOrder;
      if (ParseN2kPGN127245(correctN2kMsg, RudderPosition, Instance, RudderDirectionOrder, AngleOrder))
        log::toAll("rudder angle: " + String(RudderPosition*RADTODEG));
      else log::toAll("failed to parse rudder angle");
#endif

// process boat speed to update STW for true wind calc
void BoatSpeed(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  double SpeedWaterMeters;
  double SpeedGroundMeters; // don't really care about ground speed here; TWS and TWA should refer to boat speed thru water
  tN2kSpeedWaterReferenceType SWRT;
  if (ParseN2kPGN128259(N2kMsg, SID, SpeedWaterMeters, SpeedGroundMeters, SWRT)) {
    BoatData.STW = SpeedWaterMeters; // KEEP METERS * 1.943844; // convert m/s to kts
  }
}

// Convert apparent wind to true wind (compass i.e. TWD)
void calcTrueWindDirection() {
  double AWS = WindSensor::windSpeedMeters;
  double AWA = rotateout;
  double STW = BoatData.STW;
  double HDG = pBD->trueHeading; // Boat heading in radians (DEGREES, MUST CONVERT!)
    
  // Calculate apparent wind components relative to boat
  double aw_x = AWS * sin(AWA);  // perpendicular to boat
  double aw_y = AWS * cos(AWA);  // parallel to boat (forward/aft)
  // Calculate boat velocity components (boat moves in heading direction)
  double boat_x = STW * sin(HDG);
  double boat_y = STW * cos(HDG);
  // True wind = Apparent wind + Boat velocity
  double tw_x = aw_x + boat_x;
  double tw_y = aw_y + boat_y;
  // Calculate true wind speed and direction
  BoatData.TWS = sqrt(tw_x * tw_x + tw_y * tw_y);
  if (BoatData.TWS > BoatData.maxTWS) BoatData.maxTWS = BoatData.TWS;
  BoatData.TWA = atan2(tw_x, tw_y);
  // Normalize direction to 0-2π range
  if (BoatData.TWA < 0) {
      BoatData.TWA += 2 * PI;
  }
//#define DEBUG
#ifdef DEBUG
  Serial.printf("STW(k): %2.2f AWA(k): %2.2f TWS(k): %2.2f TWA(d): %2.2f\n", STW*1.943844, AWA*(180/M_PI), BoatData.TWS*1.943844, BoatData.TWA*(180/M_PI));
#endif
}

// calculate true wind angle (TWA) relative to the bow
void calcTrueWindAngle() {
  // Get inputs
  double AWS = WindSensor::windSpeedMeters;  // Apparent Wind Speed in m/s
  double AWA = rotateout;                    // Apparent Wind Angle in degrees
  double STW = BoatData.STW;                 // Speed Through Water in m/s
  
  // Handle edge cases
  if (AWS < 0.01) {  // Very low wind speed
    // With no apparent wind, true wind is opposite to boat motion
    BoatData.TWS = STW;
    BoatData.TWA = 180.0;
    
    // Always update maxTWS if TWS is greater
    if (BoatData.TWS > BoatData.maxTWS) {
      BoatData.maxTWS = BoatData.TWS;
    }
    
    // Calculate VMG - in this case, it's negative STW (sailing directly away from wind)
    BoatData.VMG = -STW;
    return;
  }
  
  if (STW < 0.01) {  // Very low boat speed
    // With no boat speed, apparent wind equals true wind
    BoatData.TWS = AWS;
    BoatData.TWA = AWA;
    
    // Always update maxTWS if TWS is greater
    if (BoatData.TWS > BoatData.maxTWS) {
      BoatData.maxTWS = BoatData.TWS;
    }
    
    // Still normalize the angle
    if (BoatData.TWA < 0) {
      BoatData.TWA += 360;
    } else if (BoatData.TWA >= 360) {
      BoatData.TWA -= 360;
    }
    
    // With no boat speed, VMG is zero
    BoatData.VMG = 0.0;
    return;
  }
  
  // Convert AWA from degrees to radians for trigonometric calculations
  double AWA_rad = AWA * DEGTORAD;
  
  // Calculate apparent wind components in boat-relative frame
  double aw_x = AWS * sin(AWA_rad);  // positive to starboard
  double aw_y = AWS * cos(AWA_rad);  // positive forward
  
  // Calculate true wind components by adding boat velocity
  double tw_x = aw_x;                // no change in x-component
  double tw_y = aw_y - STW;          // subtract boat speed from y-component
  
  // Calculate true wind speed
  BoatData.TWS = sqrt(tw_x * tw_x + tw_y * tw_y);
  
  // Always update maxTWS if TWS is greater or if maxTWS is near zero
  if (BoatData.TWS > BoatData.maxTWS || (BoatData.maxTWS < 0.001 && BoatData.TWS > 0.001)) {
    BoatData.maxTWS = BoatData.TWS;
  }
  
  // Calculate true wind angle
  double wind_angle_rad = atan2(tw_x, tw_y);
  
  // Convert TWA from radians to degrees
  BoatData.TWA = wind_angle_rad * RADTODEG;
  
  // Normalize TWA to 0-360 degrees range
  if (BoatData.TWA < 0) {
    BoatData.TWA += 360;
  } else if (BoatData.TWA >= 360) {
    BoatData.TWA -= 360;
  }
  
  // Calculate VMG (Velocity Made Good) to wind
  // Use the minimum angle between TWA and 360-TWA to get the absolute angle to wind
  double abs_angle_to_wind = min(BoatData.TWA, 360.0 - BoatData.TWA);
  
  // Calculate VMG: positive when sailing toward wind, negative when sailing away
  if (BoatData.TWA > 90.0 && BoatData.TWA < 270.0) {
    // Sailing away from wind (downwind)
    BoatData.VMG = -STW * cos((180.0 - abs_angle_to_wind) * DEGTORAD);
  } else {
    // Sailing toward wind (upwind)
    BoatData.VMG = STW * cos(abs_angle_to_wind * DEGTORAD);
  }

  // Calculate TWD if we have compass heading (true)
  BoatData.TWD = BoatData.TWA + pBD->trueHeading;
  
  // Normalize TWD to 0-360 degrees range
  if (BoatData.TWD < 0) {
    BoatData.TWD += 360;
  } else if (BoatData.TWD >= 360) {
    BoatData.TWD -= 360;
  }
}
