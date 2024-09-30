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
        if (!bno08x.begin_I2C()) {
            logTo::logTo::logToAll("BNO08x not found");
            //i2cScan(Wire);
            return false;
        }
        logTo::logTo::logToAll("BNO08x Found\n");
        for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
            String logString = "Part " + String(bno08x.prodIds.entry[n].swPartNumber) + ": Version :" + String(bno08x.prodIds.entry[n].swVersionMajor) + "." + String(bno08x.prodIds.entry[n].swVersionMinor) + "." + String(bno08x.prodIds.entry[n].swVersionPatch) + " Build " + String(bno08x.prodIds.entry[n].swBuildNumber);
            logTo::logTo::logToAll(logString);
        }
        return true;
    }

    void BNO085Compass::setReports(int reportType) {
        if (!bno08x.enableReport((sh2_SensorId_t)reportType)) {
            // Handle error (e.g., log or throw an exception)
        }
    }
    void BNO085Compass::setReportType(int type) {
        reportType = type;
        setReports(reportType);
    }

    void BNO085Compass::logPart() {
        logTo::logTo::logToAll("test");
    }

    float BNO085Compass::getHeading(int correction) {
        if (bno08x.wasReset()) {
            setReports(reportType);
        }

        if (!bno08x.getSensorEvent(&sensorValue)) {
            return -3.0;
        }

        switch (sensorValue.sensorId) {
            case SH2_GAME_ROTATION_VECTOR:
            case SH2_GEOMAGNETIC_ROTATION_VECTOR:
                boatAccuracy = sensorValue.un.rotationVector.accuracy;
                boatCalStatus = sensorValue.status;
                return calculateHeading(sensorValue.un.rotationVector.real, 
                                        sensorValue.un.rotationVector.i, 
                                        sensorValue.un.rotationVector.j, 
                                        sensorValue.un.rotationVector.k, 
                                        correction);
            case SH2_ARVR_STABILIZED_GRV:
                heading = calculateHeading(sensorValue.un.arvrStabilizedGRV.real, 
                                           sensorValue.un.arvrStabilizedGRV.i, 
                                           sensorValue.un.arvrStabilizedGRV.j, 
                                           sensorValue.un.arvrStabilizedGRV.k, 
                                           correction);
                return heading;
            default:
                return -4.0;
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
