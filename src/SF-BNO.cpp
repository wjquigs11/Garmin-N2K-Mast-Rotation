#ifdef SFBNO
#include "windparse.h"
#include "compass.h"
#include "boatdata.h"

#include "SparkFun_BNO08x_Arduino_Library.h"  // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x
BNO08x bno08x;

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
int reportType=0x29;

#define BNO08X_ADDR 0x78

void setupBNO() {
  Serial.begin(115200);
  
  while(!Serial) delay(10); // Wait for Serial to become available.
  // Necessary for boards with native USB (like the SAMD51 Thing+).
  // For a final version of a project that does not need serial debug (or a USB cable plugged in),
  // Comment out this while loop, or it will prevent the remaining code from running.
  
  Serial.println();
  Serial.println("BNO08x Read Example");

  Wire.begin();

  //if (bno08x.begin() == false) {  // Setup without INT/RST control (Not Recommended)
  if (bno08x.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
    Serial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1)
      ;
  }
  Serial.println("BNO08x found!");

  // Wire.setClock(400000); //Increase I2C data rate to 400kHz

  //setReports();

  Serial.println("Reading events");
  delay(100);
}

// Here is where you define the sensor outputs you want to receive
void setReports(int reportType, uint16_t timeBetweenReports) {
  Serial.printf("Setting report %d time %d\n", reportType, timeBetweenReports);
  if (bno08x.enableReport(SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR, timeBetweenReports)) {
    Serial.println(F("ARVR Stabilized Game Rotation Vector enabled"));
  } else {
    Serial.println("Could not enable ARVR Stabilized Game Rotation Vector");
  }
#if 0
  if (bno08x.enableGeomagneticRotationVector(timeBetweenReports*10) == true) {
    Serial.println(F("Geomagnetic Rotation vector enabled"));
  } else {
    Serial.println("Could not enable geomagnetic rotation vector");
  }
#endif
}

float getBNO(int correction) {
#if 0
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, timeBetweenReports);
  }
#endif
  // Has a new event come in on the Sensor Hub Bus?
  if (bno08x.getSensorEvent()) {
    int event = bno08x.getSensorEventID();
    Serial.printf("event=%d\n", event);
    if (event == SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR) {
        float heading = bno08x.getYaw()*RADTODEG;
        heading += boatOrientation;
        if (heading > 359) heading -= 360;
        if (heading < 0) heading += 360;
        return heading;
#if 0
        Serial.printf(">GR:%0.2f",bno08x.getQuatReal());
        Serial.printf(">Gi:%0.2f",bno08x.getQuatI());
        Serial.printf(">Gj:%0.2f",bno08x.getQuatJ());
        Serial.printf(">Gk:%0.2f",bno08x.getQuatK());
#endif
    }
#if 0
    if (event == SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR) {
        float heading = bno08x.getYaw()*RADTODEG;
        heading += boatOrientation;
        if (heading > 359) heading -= 360;
        if (heading < 0) heading += 360;
        return heading;
        Serial.printf(">aR:%0.2f",bno08x.getQuatReal());
        Serial.printf(">ai:%0.2f",bno08x.getQuatI());
        Serial.printf(">aj:%0.2f",bno08x.getQuatJ());
        Serial.printf(">ak:%0.2f",bno08x.getQuatK());    
    }
#endif
  }
  return -1;
}
#endif