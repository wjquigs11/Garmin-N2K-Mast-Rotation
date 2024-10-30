// BNO085Compass.h

#ifndef BNO085COMPASS_H
#define BNO085COMPASS_H

#include <Adafruit_BNO08x.h>

class BNO085Compass {
private:
    Adafruit_BNO08x bno08x;
    sh2_SensorValue_t sensorValue;

public:
    BNO085Compass(int8_t reset_pin = -1);

    int reportType;
    float heading;
    float boatAccuracy;
    int boatCalStatus;
    bool IMUready;
    bool OnToggle;
    bool teleplot=false;
    int numReports[SH2_MAX_SENSOR_ID], totalReports;

    bool begin();
    bool setReports(int reportType);
    float calculateHeading(float r, float i, float j, float k, int correction);
    void setReportType(int type);
    float getHeading(int correction);
    float getBoatAccuracy() const;
    int getBoatCalStatus() const;
    void logPart();
};

extern BNO085Compass compass;

#endif // BNO085COMPASS_H
