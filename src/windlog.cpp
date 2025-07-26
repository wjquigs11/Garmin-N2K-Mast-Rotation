#ifdef WINDLOG
/*
write wind data to log file
*/
#include "windparse.h"
#include "boatdata.h"
#include "logto.h"

bool windLogging = false;
static char logFile[32];
static char logTempName[32];
File windLogFile;
char windLog[] = "windLog";
static int lastWindLog = 0;  // Store the last update time
#define NEWLOGFILE 600000 // start a new file every 10 minutes
int windLogFileIdx;

void writeWindPoint(File &file, unsigned long timestamp, float awa, double aws, double stw, double twa, double tws, double twd, double vmg, double heading) {
  if (file) {
    file.printf("%05ld,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f\n",timestamp, awa, aws, stw, twa, tws, twd, vmg, heading);
    //log::toAll("wind point");
    if (timestamp % 300 == 0) file.flush();
  } //else log::toAll("no valid wind file");
}

void initWindLog() {
    sprintf(logTempName, "/%s%03d.log",windLog,windLogFileIdx);
    if (SPIFFS.exists(logTempName)) {
      windLogFile = SPIFFS.open(logTempName, "a", false);
    } else {
      windLogFile = SPIFFS.open(logTempName, "w", true);
      windLogFile.println("time,aws,awa,stw,tws,twa,twd,vmg,heading");
    }
    if (!windLogFile)
      log::toAll("failed to open wind log");
    else
      log::toAll("opened " + String(logTempName));
}

void startNextWindLog() {
  log::toAll("starting next wind logfile @" + String(windLogFileIdx+1) + " time: " + String(millis()));
  // close file and start a new one
  if (windLogFile) {
    ++windLogFileIdx;
    close(windLogFile);
    sprintf(logTempName, "/%s%03d.log",windLog,windLogFileIdx);
    preferences.putInt("windLogFileIdx",windLogFileIdx);
    windLogFile = SPIFFS.open(logTempName, "w", true);
    windLogFile.println("time,aws,awa,stw,tws,twa,twd,vmg,heading");
    log::toAll("opened new wind log file "+ String(logTempName));
  } else {
    log::toAll("invalid wind log file, index: " + windLogFileIdx);
  }
}
#endif