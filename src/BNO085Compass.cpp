
#ifdef BNO08X
#include <Adafruit_BNO08x.h>
#include "windparse.h"
#include "compass.h"
#include "logto.h"
#include "BNO085Compass.h"

    BNO085Compass::BNO085Compass(int8_t reset_pin) : bno08x(reset_pin), reportType(0x29) {}
    int reportType=0x29;
    float heading;
    float boatAccuracy;
    int boatCalStatus;
    bool IMUready;

    bool BNO085Compass::begin() {
        if (!bno08x.begin_I2C(BNO08X)) {
            log::log::toAll("BNO08x not found");
            //i2cScan(Wire);
            return false;
        }
        log::log::toAll("BNO08x Found\n");
        for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
            String logString = "Part " + String(bno08x.prodIds.entry[n].swPartNumber) + ": Version :" + String(bno08x.prodIds.entry[n].swVersionMajor) + "." + String(bno08x.prodIds.entry[n].swVersionMinor) + "." + String(bno08x.prodIds.entry[n].swVersionPatch) + " Build " + String(bno08x.prodIds.entry[n].swBuildNumber);
            log::log::toAll(logString);
        }
        // temporary set frequency
        frequency = 100;
        return true;
    }

    // #ifdef BNO_GRV, enable SH2_ARVR_STABILIZED_GRV (game rotation vector), as well as another report set by the user, 
    // usually SH2_GEOMAGNETIC_ROTATION_VECTOR (0x09) to get a compass heading
    // note that GRV is requested 10x the rate of the other (compass) report
    bool BNO085Compass::setReports() {
        log::toAll("Setting compass report to: 0x" + String(reportType,HEX));
        if (!bno08x.enableReport(reportType, frequency*10000)) {
            log::toAll("could not set report type: " + String(reportType,HEX));  
            return false;
        }
#ifdef BNO_GRV
        if (!bno08x.enableReport(SH2_ARVR_STABILIZED_GRV, frequency*1000)) {
            log::toAll("could not set report type (2): " + String(SH2_ARVR_STABILIZED_GRV,HEX));
            return false;
        } else log::toAll("enabled " + String(SH2_ARVR_STABILIZED_GRV,HEX));
#endif
        return true;
    }
/*        if (!bno08x.enableReport((sh2_SensorId_t)reportType)) {
            return false;
        }
        else return true;
*/

    void BNO085Compass::logPart() {
        log::log::toAll("test");
    }

    int BNO085Compass::getHeading(int correction) {
        if (bno08x.wasReset()) {
            setReports();
        }

        if (!bno08x.getSensorEvent(&sensorValue)) {
            return -3;
        }

        numReports[sensorValue.sensorId]++;
        switch (sensorValue.sensorId) {
            case SH2_GAME_ROTATION_VECTOR:
            case SH2_GEOMAGNETIC_ROTATION_VECTOR:
                boatAccuracy = sensorValue.un.rotationVector.accuracy;
                boatCalStatus = sensorValue.status;
                boatHeading = calculateHeading(sensorValue.un.rotationVector.real, 
                                        sensorValue.un.rotationVector.i, 
                                        sensorValue.un.rotationVector.j, 
                                        sensorValue.un.rotationVector.k, 
                                        correction);
#ifdef DEBUG
                log::log::toAll("BNO085 Heading: " + String(boatHeading) + " Accuracy: " + String(boatAccuracy) + " CalStatus: " + String(boatCalStatus));
#endif               
                return 1;
            case SH2_ARVR_STABILIZED_GRV:
                boatIMU = calculateHeading(sensorValue.un.arvrStabilizedGRV.real, 
                                           sensorValue.un.arvrStabilizedGRV.i, 
                                           sensorValue.un.arvrStabilizedGRV.j, 
                                           sensorValue.un.arvrStabilizedGRV.k, 
                                           correction);
                return 1;
            default:
                return -4;
        }
    }

    float BNO085Compass::getBoatAccuracy() const { return boatAccuracy; }
    int BNO085Compass::getBoatCalStatus() const { return boatCalStatus; }

    float BNO085Compass::calculateHeading(float r, float i, float j, float k, int correction) {
        float r11 = 1 - 2 * (j * j + k * k);
        float r21 = 2 * (i * j + r * k);
        float r31 = 2 * (i * k - r * j);
        float r32 = 2 * (j * k + r * i);
        float r33 = 1 - 2 * (i * i + j * j);

        float theta = -asin(r31);
        float phi = atan2(r32, r33);
        float psi = atan2(-r21, r11);
        float heading = (psi * RADTODEG) + (float)correction;

        if ((int)heading > 359) {
            heading -= 360.0;
        }
        if ((int)heading < 0) {
            heading += 360.0;
        }

        return heading;
    }
#endif