#if 0
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
#endif

#define DEGTORAD 0.01745329252
#define RADTODEG 57.2957795131
#define VARIATION -15.2
