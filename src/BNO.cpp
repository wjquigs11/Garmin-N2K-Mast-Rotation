#ifdef BNO08X
#include "windparse.h"
#include "compass.h"
#include "boatdata.h"

unsigned char high_byte, low_byte, angle8;
char pitch, roll;
unsigned int angle16, comp8;
float comp16;

extern float mastCompassDeg; 
extern float boatCompassDeg;
float boatAccuracy;
int boatCalStatus;
extern tBoatData BoatData;
extern float mastDelta;
// how is the compass oriented on the board relative to boat compass
// when mast is centered, mast compass+orientation == boat compass
extern int mastOrientation, sensOrientation, boatOrientation;
extern int compassFrequency;
extern bool compassOnToggle;
//extern JSONVar readings;

extern AsyncWebServer server;
extern AsyncEventSource events;

float getCompass(int correction);
void logToAll(String message);
void i2cScan(TwoWire Wire);

// for sending mast compass on bus
extern tNMEA2000 *n2kMain;
extern tN2kMsg correctN2kMsg;

int reportType=0x29;

extern bool teleplot;

#ifdef CMPS14
extern const int CMPS14_ADDRESS;  // Address of CMPS14 shifted right one bit for arduino wire library
#define ANGLE_8  1          // Register to read 8bit angle from (starting...we read 5 bytes)
#define CAL_STATE 0x1E      // register to read calibration state
byte calibrationStatus[8];
bool readCalibrationStatus();
// get heading from (local) compass
// called when we get a Heading PGN on the wind bus (which means mast compass transmitted)
// so frequency of update is going to depend on how often we get a Heading PGN from the mast
// also called as a Reaction in case we're not connected to mast compass
// TBD: make this object oriented and overload getCompass for either CMPS14 or BNO085
float getCMPS14(int correction) {
  //Serial.println("getCompass");
  Wire.beginTransmission(CMPS14_ADDRESS);  // starts communication with CMPS14
  Wire.write(ANGLE_8);                     // Sends the register we wish to start reading from
  Wire.endTransmission();
  // Request 5 bytes from the CMPS12
  // this will give us the 8 bit bearing, 
  // both bytes of the 16 bit bearing, pitch and roll
  Wire.requestFrom(CMPS14_ADDRESS, 5); 
  while((Wire.available() < 5)); // (this can hang?)
  angle8 = Wire.read();               // Read back the 5 bytes
  comp8 = map(angle8, 0, 255, 0, 359);
  //comp8 = (comp8 + correction + 360) % 360;
  high_byte = Wire.read();
  low_byte = Wire.read();
  pitch = Wire.read();
  roll = Wire.read();
  angle16 = (high_byte <<8) + low_byte;                 // Calculate 16 bit angle
  comp16 = (angle16/10) + (angle16%10)/10.0;
  comp16 += correction;
#ifdef DEBUG
    Serial.print("angle8: "); Serial.print(angle8, DEC);
    Serial.print(" comp8: "); Serial.print(comp8, DEC);
    Serial.print(" angle 16: ");     // Display 16 bit angle with decimal place
    Serial.print(angle16/10, DEC);
    Serial.print(".");
    Serial.print(angle16%10, DEC);
    Serial.print(" a16: "); Serial.printf("%0.2f", (angle16/10) + (angle16%10)/10.0);
    Serial.print(" corr: "); Serial.println(correction);
#endif
  if (comp16 > 359) comp16 -= 360;
  if (comp16 < 0) comp16 += 360;
  if (teleplot) {
    Serial.print(">mast: "); Serial.println(comp16); 
  }
  Wire.beginTransmission(CMPS14_ADDRESS);  // starts communication with CMPS14
  Wire.write(CAL_STATE);                     // Sends the register we wish to start reading from
  Wire.endTransmission();
  Wire.requestFrom(CMPS14_ADDRESS, 5); 
  while((Wire.available() < 1)); // (this can hang)
  boatCalStatus = Wire.read() & 0x3;
  readCalibrationStatus();
  ////Serial.printf("calStatus: 0x%x\n", boatCalStatus);
  //logToAll("heading: " + String(comp16) + " cal stat: " + String(boatCalStatus));
  return comp16;
}

bool readCalibrationStatus() {
  ////Serial.print("read cal status");
  Wire.beginTransmission(CMPS14_ADDRESS);
  Wire.write(CAL_STATE);
  int nackCatcher = Wire.endTransmission();
  if (nackCatcher != 0) return false;
  // Request 1 byte from CMPS14
  int nReceived = Wire.requestFrom(CMPS14_ADDRESS, 1);
  if (nReceived != 1) return false;
  byte Byte = Wire.read();
  for (int i = 0; i < 8; i++) {
    bool b = Byte & 0x80;
    if (b) {
      calibrationStatus[i] = 1;
    } else {
      calibrationStatus[i] = 0;
    }
    Byte = Byte << 1;
  }
  return true;
}
#endif // CMPS14

float calculateHeading(float r, float i, float j, float k, int correction);

static float heading;

#define BNO08X //temp!!!
#ifdef BNO08X
#include <Adafruit_BNO08x.h>
Adafruit_BNO08x bno08x(RESET);
sh2_SensorValue_t sensorValue;

// enabling whatever report the user requested, plus SH2_GEOMAGNETIC_ROTATION_VECTOR for heading
void setReports(int reportType) {
  logToAll("Setting compass report to: 0x" + String(reportType,HEX));
  if (!bno08x.enableReport((sh2_SensorId_t)reportType))
    logToAll("could not set report type: " + String(reportType,HEX));  
#if 0 // doesn't look practical to enable two report types at once
  if (!bno08x.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR))
    logToAll("could not set report type: SH2_GEOMAGNETIC_ROTATION_VECTOR");
  else logToAll("enabled SH2_GEOMAGNETIC_ROTATION_VECTOR");
#endif
}

float getBNO085(int correction) {
  if (bno08x.wasReset()) {
    setReports(reportType); // ugh...global
  }
  if (!bno08x.getSensorEvent(&sensorValue)) {
    return -3.0;
  }
  /* Status of a sensor
   *   0 - Unreliable
   *   1 - Accuracy low
   *   2 - Accuracy medium
   *   3 - Accuracy high
   */
  float quatRadianAccuracy, yaw;
  int sensAccuracy;

  switch (sensorValue.sensorId) {
  case SH2_GAME_ROTATION_VECTOR:
  case SH2_GEOMAGNETIC_ROTATION_VECTOR:
    boatAccuracy = sensorValue.un.rotationVector.accuracy;
    boatCalStatus = sensorValue.status;
    //logToAll("heading: " + String(heading) + " accuracy: " + String(boatAccuracy) + " cal status: " + boatCalStatus + " size: " + String(sizeof(sensorValue)));
    // if we get this report we're going to set boat true but not change boatCompassDeg
    BoatData.TrueHeading = BoatData.Variation + calculateHeading(sensorValue.un.rotationVector.real, sensorValue.un.rotationVector.i, sensorValue.un.rotationVector.j, sensorValue.un.rotationVector.k, correction);      
;
    if (BoatData.TrueHeading > 359) BoatData.TrueHeading -= 360;
    if (BoatData.TrueHeading < 0) BoatData.TrueHeading += 360;
    //return heading;
    //break;
  case SH2_ARVR_STABILIZED_GRV:
    //boatCalStatus = sensorValue.status;
    heading = calculateHeading(sensorValue.un.arvrStabilizedGRV.real, sensorValue.un.arvrStabilizedGRV.i, sensorValue.un.arvrStabilizedGRV.j, sensorValue.un.arvrStabilizedGRV.k, correction);      
    //logToAll("heading: " + String(heading) + " accuracy: " + String(boatAccuracy) + " cal status: " + boatCalStatus + " size: " + String(sizeof(sensorValue)));
    return heading;
    break;
  case SH2_GYROSCOPE_CALIBRATED:
    logToAll("gyroscope calibrated");
  /*
    //Serial.print("Gyro - x: ");
    //Serial.print(sensorValue.un.gyroscope.x);
    //Serial.print(" y: ");
    //Serial.print(sensorValue.un.gyroscope.y);
    //Serial.print(" z: ");
    //Serial.println(sensorValue.un.gyroscope.z);
    */
    ////Serial.printf("1 heading %0.2f\n", heading);
    //return heading;
    break;
  case SH2_MAGNETIC_FIELD_CALIBRATED:
    logToAll("magnetic field calibrated");
    /*//Serial.print("Magnetic Field - x: ");
    //Serial.print(sensorValue.un.magneticField.x);
    //Serial.print(" y: ");
    //Serial.print(sensorValue.un.magneticField.y);
    //Serial.print(" z: ");
    //Serial.println(sensorValue.un.magneticField.z);
    */
    ////Serial.printf("2 heading %0.2f\n", heading);
    //boatCalStatus = sensorValue.status;
    //return heading;
    break;
  default:
    logToAll("getBNO085() got unknown sensor id: 0x" + String(sensorValue.sensorId, HEX));
    return -4.0;
    break;
  }
  return -5.0;
}
#endif

// Function to calculate tilt-compensated heading from a quaternion
float calculateHeading(float r, float i, float j, float k, int correction) {
  //Serial.printf("r %0.2f i %0.2f j %0.2f k %0.2f %2d ", r, i, j, k, correction);
  // Convert quaternion to rotation matrix
  float r11 = 1 - 2 * (j * j + k * k);
  // change r21 to =-2 if sensor is upside down
  float r21 = 2 * (i * j + r * k);
  float r31 = 2 * (i * k - r * j);
  float r32 = 2 * (j * k + r * i);
  float r33 = 1 - 2 * (i * i + j * j);
  ////Serial.printf("r11 %0.2f r21 %0.2f r31 %0.2f r32 %0.2f r33 %0.2f ", r11, r21, r31, r32, r33);
  // Calculate pitch (theta) and roll (phi)
  float theta = -asin(r31);
  float phi = atan2(r32, r33);
  // Calculate yaw (psi)
  float psi = atan2(-r21, r11);
  float heading = (psi * RADTODEG) + (float)correction;
  ////Serial.printf("theta %0.2f phi %0.2f psi %0.2f h %0.2f ", theta, phi, psi, heading);
  ////Serial.printf("h %0.2f ", heading);
  // correction may be positive or negative
  if ((int)heading > 359) {
    heading -= 360.0;
  }
  if ((int)heading < 0) {
    heading += 360.0;
  }
  //Serial.printf("h %0.2f\n", heading);
  return heading;
}

#if 0
// Calculate pitch (theta) and roll (phi)
  float theta = -asin(r31);
  float phi = atan2(r32, r33);

  // Calculate yaw (psi)
  // Change the sign of r21 to reverse the direction
  float psi = atan2(-r21, r11);

  // Convert to degrees and apply correction
  float heading = (psi * 180 / M_PI) + correction;

  // Normalize heading to 0-360 range
  heading = fmod(heading, 360);
  if (heading < 0) heading += 360;

  return heading;
#endif

#if 0
//#define XMITMAST  // send mast compass on main bus
// get heading from MASTIMU
// not using magnetic sensor so not referenced to north
float getCompass(int correction) {
  //Serial.printf("getcompass corr %d ", correction);
  float compassVal;
  if (!imuReady)
    return -1;
#ifdef CMPS14
  compassVal = getCMPS14(correction);
#elif defined(MASTIMU)
  compassVal = getBNO085(correction);
  ////Serial.printf("bno085 %0.2f\n", compassVal);
#endif
#ifdef XMITMAST
  //SetN2kPGN127250(correctN2kMsg, 0xFF, (double)compassVal*DEGTORAD, N2kDoubleNA, N2kDoubleNA, N2khr_magnetic);
  SetN2kPGN127245(correctN2kMsg, compassVal*DEGTORAD, 0, N2kRDO_NoDirectionOrder, N2kDoubleNA);
  bool result = n2kMain->SendMsg(correctN2kMsg);
  if (!result) Serial.println("Failed to send mast heading message as rudder");
  else logToAll("send mast heading rudder " + String(compassVal));
#endif
  return compassVal;
}
#endif

/*
https://www.arduino.cc/reference/en/language/functions/communication/wire/endtransmission/
endTransmission() returns:
0: success.
1: data too long to fit in transmit buffer.
2: received NACK on transmit of address.
3: received NACK on transmit of data.
4: other error.
5: timeout
*/

#if 0
// called when we get a Mag Heading message from bus (i.e. from RPI, boat heading)
// checks mast heading (async) and prints difference (in degrees)
// RETURNS difference, if you want to xmit rudder angle add 50 (if mast range is -50..50)
// WILL NOT be called if there's a compass connected to ESP32 (local compass)
int convertMagHeading(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  double heading;
  double deviation;
  double variation;
  tN2kHeadingReference headingRef;
  tN2kMsg correctN2kMsg; 

  //N2kMsg.Print(&Serial);
  if (ParseN2kPGN127250(N2kMsg, SID, heading, deviation, variation, headingRef) ) {
    #ifdef DEBUG2
    //Serial.print("SID: "); //Serial.print(SID);
    //Serial.print(" heading: "); //Serial.print(heading);
    //Serial.print(" deviation: "); //Serial.print(deviation);
    //Serial.print(" variation: "); //Serial.print(variation);
    //Serial.print(" headingRef: "); //Serial.print(headingRef);
    //Serial.println();
    #endif
    if (headingRef == N2khr_magnetic && heading > 0) {  // need to check heading because it could be null
      ////Serial.print("\tcompass heading: ");
      mastCompassDeg = heading * (180/M_PI);
      //readings["heading"] = String(mastCompassDeg);
      ////Serial.print(headingDeg);
      // compare to heading reading from mast compass (via wifi)
      ////Serial.print(" mast bearing: ");
      ////Serial.print(mastOrientation);
      ////Serial.print(" difference: ");
      int delta = mastOrientation - mastCompassDeg;
      if (delta > 180) {
        delta -= 360;
      } else if (delta < -180) {
        delta += 360;
      }
      readings["mastDelta"] = String(delta); // TBD shift range to 0..100 for gauge
      mastAngle[1] = delta;
      // tbd add mastcompdelta
      ////Serial.println(mastAngle[1]);
      return delta;
    } else {
      //Serial.println("no magnetic heading from main compass; doing nothing");
      return -1;
    }
  }
  return -1;
}
#endif

#if 0
void parse_mast_comp() {
JSONVar myObject = JSON.parse(jsonString);

  // Check if parsing was successful
  if (JSON.typeof(myObject) == "undefined") {
    //Serial.println("Parsing input failed!");
    return;
  }

  // Access the values
  if (myObject.hasOwnProperty("bearing")) {
    double bearing = myObject["bearing"];
    //Serial.print("Bearing: ");
    //Serial.println(bearing);
  }

  if (myObject.hasOwnProperty("calstatus")) {
    int calstatus = (int)myObject["calstatus"];
    //Serial.print("Calstatus: ");
    //Serial.println(calstatus);
  }
#endif
#endif