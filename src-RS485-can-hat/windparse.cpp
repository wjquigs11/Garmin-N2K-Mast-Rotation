// parse wind speed, correct for mast rotation

#include <Arduino.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>
#include <N2kMessages.h>
#include <NMEA2000_esp32.h>
#include <ReactESP.h>
#include <Wire.h>
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>
#include <vector>
#include <numeric>
#include <movingAvg.h>
//#include <SPI.h>
#include "windparse.h"
//#include <driver/adc.h>
#include <Adafruit_ADS1X15.h>
#include <Arduino_JSON.h>
//#include "mcp2515.h"
//#include "can.h"
#include "BoatData.h"
#include "mcp2515_can.h"

double rotateout;
// analog values from rotation sensor
int PotValue=0;
int PotLo=9999;
int PotHi=0;
float mastRotate;
extern int portRange, stbdRange; // NB BOTH are positive (from web calibration)
extern int mastAngle[]; // range from -50 to +50, TBD set range in calibration
extern bool compassOnToggle, honeywellOnToggle;
extern float mastCompassDeg, boatCompassDeg, mastDelta;
extern int mastOrientation;   // delta between mast compass and boat compass
float getCompass(int correction);
extern tBoatData BoatData;

// Initialize static variables for RotationSensor Class
int RotationSensor::newValue{0};
int RotationSensor::oldValue{0};

// not sure if I need these any more since I'm doing it all in one function
double WindSensor::windSpeedKnots{0.0};
double WindSensor::windAngleDegrees{0.0};
double WindSensor::windSpeedMeters{0.0};
double WindSensor::windAngleRadians{0.0};
double SpeedThruWater; // meters/sec
double TWS; // meters/sec
int TWA; // radians

movingAvg honeywellSensor(10);
#ifndef PICANM
Adafruit_ADS1015 ads;
int adsInit;
#endif

extern tNMEA2000 *n2kMain;
extern int num_wind_messages;

extern JSONVar readings;

#define PRBUF 256
char prbuf[PRBUF];

void calcTrueWind();

// returns degrees, and corresponds to the current value of the Honeywell sensor
float readAnalogRotationValue() {      
  // Define Constants
  const int lowset = 56;
  const int highset = 311;
#if defined(PICANM) || defined(SH_ESP32)
// TBD: check whether you put an ADC into the controller on the boat
  PotValue = analogRead(POT_PIN);
#endif
#ifdef ESPBERRY
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
  readings["PotValue"] = String(PotValue);

  #ifdef DEBUG
  sprintf(buf, " pot(l/v/h): %d/%d/%d ", PotLo, PotValue, PotHi);
  Serial.print(buf);
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
  mastAngle[0] = map(oldValue, lowset, highset, -portRange*10, stbdRange*10);    
  readings["mastRotate"] = mastAngle[0];
  #ifdef DEBUG2
  sprintf(buf, "%d %d %d %d", oldValue, lowset, highset, mastRot);
  Serial.println(buf);
  #endif
  return mastAngle[0]; 
}

//long unsigned int rxId;
//unsigned char len;
//unsigned char rxBuf;

#define DEBUGCAN2
extern mcp2515_can n2kWind;
byte canRx(byte cPort, unsigned long* lMsgID, byte* cMessageIDFormat, byte* cData, byte* cDataLen);

#ifdef PICANM
void WindSpeed(const tN2kMsg &N2kMsg) {};
#endif

void ParseWindCAN() {
  double windSpeedMeters;
  double windSpeedKnots;
  double windAngleRadians;
  int windAngleDegrees;
  unsigned long t = millis();
  int PGN;
  tN2kMsg correctN2kMsg;
  byte result;
  byte cRetCode = CAN_NOMSG;
  uint32_t id;
  uint8_t  type; // bit0: ext, bit1: rtr
  uint8_t  cDatalen;
  byte cdata[MAX_DATA_SIZE] = {0};
  //struct can_frame canMsg;
  //byte cDataLen;
  //byte cData[8];
  num_wind_messages++;

  // Check for dataframe at CAN0
  // Read the message
  //if ((cRetCode = n2kWind.readMessage(&canMsg)) == CAN_OK) {
  if (n2kWind.checkReceive() != CAN_MSGAVAIL)
    return;
  if ((cRetCode = n2kWind.readMsgBuf(&cDatalen, cdata)) == CAN_OK) {
    id = n2kWind.getCanId();
    type = (n2kWind.isExtendedFrame() << 0) | (n2kWind.isRemoteRequest() << 1);
    PGN = ((id & 0x1FFFFFFF)>>8) & 0x3FFFF; // mask 00000000000000111111111111111111
    //#ifdef DEBUG
    Serial.printf("can PGN: %d LEN: %d retcode: %d\n", PGN, cDatalen, cRetCode);
    //#endif
  } else {
    // 5 is not an error just means no data on CAN bus
    if (cRetCode != 5)
      Serial.printf("wind CAN read error: %d\n", cRetCode);
    return;
  }
  // wind PGN
  if (PGN == 130306) { 
//    int SID = canMsg.data[0];
    int SID = cdata[0];
    double windSpeed = ((cdata[2] << 8) | cdata[1]);
    windSpeedMeters = windSpeed / 100;
    windSpeedKnots =  windSpeedMeters * 1.943844; // convert m/s to kts
    double wAngle = ((cdata[4] << 8) | cdata[3]);
    float windAngleRadians = wAngle / 10000.0;
    float windAngleDegrees = windAngleRadians * (180/M_PI);
    uint8_t wRef = cdata[5];
    #ifdef DEBUG
    snprintf(prbuf, PRBUF, "SID: %d speed(kts): %2.2f angle %2.2f/%2.2f ref %d", SID, windSpeedKnots, wAngle, windAngleDegrees, wRef);
    Serial.println(prbuf);
    #endif
    WindSensor::windSpeedMeters = windSpeedMeters;
    WindSensor::windSpeedKnots = windSpeedKnots;
    readings["windSpeed"] = String(windSpeedKnots);
    WindSensor::windAngleRadians = windAngleRadians;
    WindSensor::windAngleDegrees = windAngleDegrees;
    readings["windAngle"] = String(windAngleDegrees);
    if (wRef != 2) { // N2kWind_Apparent
      Serial.printf("got wind PGN not apparent! %d\n", wRef);
      return;
    }
    // read rotation value and correct
    // either read from Honeywell sensor, or from external compass, or both
    // if external compass, reading will be updated in espnow.cpp
    mastRotate = 0.0;
    if (honeywellOnToggle) {
      mastRotate = readAnalogRotationValue();
      //Serial.printf("honeywell mastrotate = %d\n", (int)mastRotate);
    }
    if (compassOnToggle) {
      readings["mastHeading"] = String(mastCompassDeg);
      readings["boatHeading"] = String(boatCompassDeg);
      readings["mastDelta"] = String(mastDelta);
      //Serial.printf("ParseWindCAN mast %.2f boat %.2f mastrotate = %.2f\n", mastCompassDeg, boatCompassDeg, delta);
      // if both are enabled/present, use Honeywell
      if (!honeywellOnToggle)
        mastRotate = mastDelta;
    }
    #ifdef DEBUG
    //Serial.print(" mastRotate: ");
    //Serial.println(mastRotate);
    #endif
    double anglesum = windAngleDegrees + mastRotate;    // sensor AFT of mast so subtract rotation
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
    readings["rotateout"] = String(rotateout);
    #ifdef DEBUG
    snprintf(prbuf, PRBUF, "rotate now %d low %d high %d", PotValue, PotLo, PotHi);
    Serial.println(prbuf);
    snprintf(prbuf, PRBUF, "mastRotate %d anglesum %d rotateout %d", mastRotate, anglesum, rotateout);
    Serial.println(prbuf);
    #endif
    // send corrected wind on main bus
    // note we are sending the original speed reading in m/s
    // and the AWA converted from rads to degrees, corrected, and converted back to rads
    SetN2kPGN130306(correctN2kMsg, 0xFF, windSpeedMeters, rotateout*(M_PI/180), N2kWind_Apparent); 
    n2kMain->SendMsg(correctN2kMsg);
    // for now (until you dive into SensESP), send rotation angle as rudder
    SetN2kPGN127245(correctN2kMsg, (mastRotate+50)*(M_PI/180), 0, N2kRDO_NoDirectionOrder, 0);
    n2kMain->SendMsg(correctN2kMsg);
    // calculate TWS/TWA from boat speed and send another wind PGN
    calcTrueWind();
    SetN2kPGN130306(correctN2kMsg, 0xFF, TWS, TWA, N2kWind_True_water);
    n2kMain->SendMsg(correctN2kMsg);
  } else if (PGN == 127250) {  // heading PGN; on wind bus must come from mast compass
    // TBD break this into 2 functions
    // update mast heading
    double mastComp;
    int SID = cdata[0];
    mastComp = ((cdata[2] << 8) | cdata[1]) / 10000.0; // radians
    // TBD get "reference" to confirm it's N2khr_Unavailable
    if (mastComp > -1) {
      #ifdef DEBUG
      Serial.printf("mastComp: %.2f, deg %.2f\n", mastComp, mastComp * 57.296);
      #endif
      mastCompassDeg = mastComp * 57.296;
      // we get boatCompassDeg here but we should also do it on schedule so the ship's compass is still valid even if we're not connected to mast compass
      boatCompassDeg = getCompass(mastOrientation);
      mastDelta = mastCompassDeg-boatCompassDeg;
      if (mastDelta > 180) {
        mastDelta -= 360;
      } else if (mastDelta < -180) {
        mastDelta += 360;
      }
      mastAngle[1] = mastDelta; 
    }   
    //Serial.printf("heading PGN Mast: %.2f Boat: %.2f\n", mastCompassDeg, boatCompassDeg);
  } else {
    Serial.printf("unknown PGN: %d LEN: %d\n", PGN, cDatalen);
  }
}

// process boat speed to update STW for true wind calc
void BoatSpeed(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  double SpeedWaterMeters;
  double SpeedGroundMeters; // don't really care about ground speed here; TWS and TWA should refer to boat speed thru water
  tN2kSpeedWaterReferenceType SWRT;
  tN2kMsg correctN2kMsg;
  if (ParseN2kPGN128259(N2kMsg, SID, SpeedWaterMeters, SpeedGroundMeters, SWRT)) {
    SpeedThruWater = SpeedWaterMeters; // KEEP METERS * 1.943844; // convert m/s to kts
    BoatData.STW = SpeedThruWater;
  }
}

void calcTrueWind() {
  // note using meters/sec and radians
  //Serial.printf("WS(m): %2.2f WA(r): %2.2f cos: %2.2f\n", WindSensor::windSpeedMeters, WindSensor::windAngleRadians, cos(WindSensor::windAngleRadians));
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
  Serial.printf("STW(k): %2.2f TWS(k): %2.2f TWA(d): %2.2f\n", STW*1.943844, TWS*1.943844, TWA*(180/M_PI));
#endif
}




