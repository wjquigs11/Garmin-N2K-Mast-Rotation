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

double rotateout;
// analog values from rotation sensor
int PotValue=0;
int PotLo=9999;
int PotHi=0;
extern int portRange, stbdRange; // NB BOTH are positive (from web calibration)
extern int mastAngle[]; // range from -50 to +50, TBD set range in calibration
extern bool compassOnToggle, honeywellOnToggle;
extern float mastCompassDeg, boatCompassDeg;

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
Adafruit_ADS1015 ads;
int adsInit;

extern tNMEA2000 *n2kMain;
//extern HardwareSerial ESPlink;
extern int num_wind_messages;

extern JSONVar readings;
//extern JsonArray readings;

#define PRBUF 64
char buf[PRBUF];

/*
extern volatile bool new_data;
void IRAM_ATTR NewDataReadyISR() {
  new_data = true;
}
*/

void calcTrueWind();

// returns degrees (x10), and corresponds to the current value of the Honeywell sensor
int readAnalogRotationValue() {      
  // Define Constants
  const int lowset = 56;
  const int highset = 311;
  
  //PotValue = analogRead(POT_PIN);
  if (adsInit)
    PotValue = ads.readADC_SingleEnded(0);
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
  int newValue = honeywellSensor.reading(PotValue);    // calculate the moving average
  int oldValue = RotationSensor::oldValue;
  
  if (newValue < highset) {  // writes value to oldsensor if below highset threshold
    oldValue = newValue;
  }
  // Update values for new and old values (for the next loop iteration)
  RotationSensor::newValue = newValue;
  RotationSensor::oldValue = oldValue;

  // map 10 bit number to degrees of rotation (*10 for precision)
  mastAngle[0] = map(oldValue, lowset, highset, -portRange*10, stbdRange*10);    
  readings["mastRotate"] = mastAngle[0]/10;
  #ifdef DEBUG2
  sprintf(buf, "%d %d %d %d", oldValue, lowset, highset, mastRot);
  Serial.println(buf);
  #endif
  return mastAngle[0]; 
}

void WindSpeed(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  double windSpeedMeters;
  double windSpeedKnots;
  double windAngleRadians;
  double windAngleDegrees;
  tN2kWindReference WindReference;
  tN2kMsg correctN2kMsg;
  int mastRotate;
//#define DEBUG
  if (ParseN2kWindSpeed(N2kMsg,SID, windSpeedMeters, windAngleRadians, WindReference)) {
    if (WindReference != N2kWind_Apparent) {
      Serial.printf("got wind PGN not apparent! %d\n", WindReference);
      return;
    }
    WindSensor::windSpeedMeters = windSpeedMeters;
    windSpeedKnots =  windSpeedMeters * 1.943844; // convert m/s to kts
    WindSensor::windSpeedKnots = windSpeedKnots;
    readings["windSpeed"] = String(windSpeedKnots);
    WindSensor::windAngleRadians = windAngleRadians;
    windAngleDegrees = windAngleRadians*(180/M_PI);
    WindSensor::windAngleDegrees = windAngleDegrees;
    readings["windAngle"] = String(windAngleDegrees);
  #ifdef DEBUG
    Serial.printf("wind reference %d ", WindReference);
    Serial.print("windSpeedKnots: ");
    Serial.print(windSpeedKnots);
    Serial.print(" windangledegrees: ");
    Serial.print(windAngleDegrees);
    #endif
    // read rotation value and correct
    // either read from Honeywell sensor, or from external compass, or both
    // if external compass, reading will be updated in espnow.cpp
    mastRotate = 0;
    if (honeywellOnToggle) {
      mastRotate = readAnalogRotationValue();
      Serial.printf("honeywell mastrotate = %d\n", mastRotate);
    }
    if (compassOnToggle) {
      float delta = mastCompassDeg-boatCompassDeg;
      if (delta > 180) {
        delta -= 360;
      } else if (delta < -180) {
        delta += 360;
      }
      mastAngle[1] = (int)delta;
      readings["mastHeading"] = String(mastCompassDeg);
      readings["boatHeading"] = String(boatCompassDeg);
      readings["mastDelta"] = String(delta);
      //Serial.printf("WindSpeed mast %.2f boat %.2f mastrotate = %d\n", mastCompassDeg, boatCompassDeg, delta);
      // TBD: decide which one to use
      //mastRotate = delta;
    }
    #ifdef DEBUG
    Serial.print(" mastRotate: ");
    Serial.print(mastRotate);
    #endif
    // still using Honeywell; TBD make configurable
    double anglesum = windAngleDegrees + mastRotate/10;
    if (anglesum<0) { // ensure sum is 0-359
      rotateout = anglesum + 360; 
    } else if (anglesum>359) {   
      rotateout = anglesum - 360;               
    } else {
      rotateout = anglesum;               
    }
    readings["rotateout"] = String(rotateout);
    #ifdef DEBUG
    Serial.print(" rotateout: ");
    Serial.print(rotateout);
    Serial.print(" radians: ");
    Serial.println(rotateout*(M_PI/180));
    #endif
    #ifdef DEBUG2
    Serial.println("sending corrected wind on n2kMain");
    #endif
    // send corrected wind
    SetN2kPGN130306(correctN2kMsg, 0xFF, windSpeedMeters, rotateout*(M_PI/180), N2kWind_Apparent); 
    n2kMain->SendMsg(correctN2kMsg);
    // for now (until you dive into SensESP), send rotation angle as rudder
    SetN2kPGN127245(correctN2kMsg, ((mastRotate/10)+50)*(M_PI/180), 0, N2kRDO_NoDirectionOrder, 0);
    n2kMain->SendMsg(correctN2kMsg);
    // calculate TWS/TWA from boat speed and send another wind PGN
    calcTrueWind();
    SetN2kPGN130306(correctN2kMsg, 0xFF, TWS, TWA, N2kWind_True_water);
    n2kMain->SendMsg(correctN2kMsg);
   } 
}

void BoatSpeed(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  double SpeedWaterMeters;
  double SpeedGroundMeters; // don't really care about ground speed here; TWS and TWA should refer to boat speed thru water
  tN2kSpeedWaterReferenceType SWRT;
  tN2kMsg correctN2kMsg;
  if (ParseN2kPGN128259(N2kMsg, SID, SpeedWaterMeters, SpeedGroundMeters, SWRT)) {
    SpeedThruWater = SpeedWaterMeters; // KEEP METERS * 1.943844; // convert m/s to kts
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
#define DEBUG
#ifdef DEBUG
  Serial.printf("STW(k): %2.2f TWS(k): %2.2f TWA(d): %2.2f\n", STW*1.943844, TWS*1.943844, TWA*(180/M_PI));
#endif
}




