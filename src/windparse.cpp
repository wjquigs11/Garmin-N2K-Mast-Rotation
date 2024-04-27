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

int mastRotate, rotateout;
// analog values from rotation sensor
int PotValue=0;
int PotLo=9999;
int PotHi=0;
extern int portRange, stbdRange; // NB BOTH are positive (from web calibration)

// Initialize static variables for RotationSensor Class
int RotationSensor::newValue{0};
int RotationSensor::oldValue{0};

// not sure if I need these any more since I'm doing it all in one function
double WindSensor::windSpeedKnots{0.0};
int WindSensor::windAngleDegrees{0};

movingAvg honeywellSensor(10);
Adafruit_ADS1015 ads;

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

// returns degrees, and corresponds to the current value of the Honeywell sensor
int readAnalogRotationValue() {      
  // Define Constants
  const int lowset = 56;
  const int highset = 311;
  
  //PotValue = analogRead(POT_PIN);
  PotValue = ads.readADC_SingleEnded(0);
  //int AltValue = adc1_get_raw(ADC1_CHANNEL_5);
  if (!PotValue)
    return 0;
  // determine range of A2D values; this might be different on your boat
  // after calibration, a value lower than PotLo or higher than PotHi should cause an error
  if (PotValue < PotLo && PotValue > 0) { 
    PotLo = PotValue;
  } else if (PotValue > PotHi) {
    PotHi = PotValue;
  }
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

  int mastRot = map(oldValue, lowset, highset, -portRange, stbdRange);    // maps 10 bit number to degrees of rotation
  #ifdef DEBUG2
  sprintf(buf, "%d %d %d %d", oldValue, lowset, highset, mastRot);
  Serial.println(buf);
  #endif
  return mastRot; 
}

void WindSpeed(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  double windSpeedMeters;
  double windSpeedKnots;
  double windAngleRadians;
  int windAngleDegrees;
  tN2kWindReference WindReference;
  tN2kMsg correctN2kMsg; // can this be a local?

  if (ParseN2kWindSpeed(N2kMsg,SID, windSpeedMeters, windAngleRadians, WindReference) ) {
    windSpeedKnots =  windSpeedMeters * 1.943844; // convert m/s to kts
    float windAngleDegrees = windAngleRadians * (180/M_PI);
    WindSensor::windSpeedKnots = windSpeedKnots;
    WindSensor::windAngleDegrees = windAngleDegrees;
    // set up readings for html page
    readings["windSpeed"] = String(windSpeedKnots);
    readings["windAngle"] = String(windAngleDegrees);
    #ifdef DEBUG
    Serial.print("windSpeedKnots: ");
    Serial.print(windSpeedKnots);
    Serial.print(" windangledegrees: ");
    Serial.print(windAngleDegrees);
    #endif
    // read rotation value and correct
    mastRotate = readAnalogRotationValue();
    readings["mastRotate"] = String(mastRotate);
    #ifdef DEBUG
    Serial.print(" mastRotate: ");
    Serial.print(mastRotate);
    #endif
    float anglesum = windAngleDegrees + mastRotate;
    int rotateout=0;
    if (anglesum<0) { // ensure sum is 0-359
      rotateout = anglesum + 360; 
    } else if (anglesum>359) {   
      rotateout = anglesum - 360;               
    } else {
      rotateout = anglesum;               
    }
    readings["rotateout"] = rotateout;
    #ifdef DEBUG
    Serial.print(" rotateout: ");
    Serial.println(rotateout);
    #endif
    #ifdef DEBUG2
    Serial.println("sending corrected wind on n2kMain");
    #endif
    SetN2kPGN130306(correctN2kMsg, 0xFF, windSpeedMeters, rotateout*(M_PI/180), N2kWind_Apparent); 
    //correctN2kMsg.Print();
    n2kMain->SendMsg(correctN2kMsg);
    // for now (until you dive into SensESP), send rotation angle as rudder
    SetN2kPGN127245(correctN2kMsg, mastRotate*(M_PI/180), 0, N2kRDO_NoDirectionOrder, 0);
    n2kMain->SendMsg(correctN2kMsg);
   } 
}
