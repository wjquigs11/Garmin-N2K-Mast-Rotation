typedef struct compass_s {
    int id; // report type
    float heading;
    float accuracy;
    int calStatus;
    int readingId;
    bool compassOnToggle = true;
    int orientation = 0;
    int variation = 0;
    int frequency = 100;
    char hostname[64] = "";
  } compass_s;
extern compass_s compassParams;

// also send sh2_SensorValue_t sensorValue for raw data

#define BNOREADRATE 20 // msecs for 50Hz rate; optimum for BNO08x calibration

#define DEGTORAD 0.01745329252
#define RADTODEG 57.2957795131
#define VARIATION -15.2

#define BNO08X_INT  -1
#define BNO08X_RST  -1

#define WIRE_PORT Wire // desired Wire port.
#define AD0_VAL 1      // value of the last bit of the I2C address.
// default I2C address = 0x69

#define RESET -1