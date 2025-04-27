#ifdef GPX
/*
write .gpx files to record track
note: gpx file needs to have separate sections for waypoints and trackpoints
So I will open two files, and append the trackpoint file to the end of the waypoints file
when the waypoint file is closed

looks like Viking MIGHT support display of trackpoint extensions
https://github.com/viking-gps/viking
but maybe only for colors
*/
#include "windparse.h"
#include "boatdata.h"
#include "logto.h"

char logFile[32];
char logTempName[32];
File GPXlogFile, WPTlogFile;
char GPXlog[32];
int lastLogFile = 0;  // Store the last update time
#define NEWLOGFILE 600000 // start a new .gpx file every 10 minutes
int logFileIdx;

void writeGPXHeader(File &file) {
    file.print(F("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"));
    file.print(F("<gpx version=\"1.1\" creator=\"ESP GPS Tracker\" "));
    file.print(F("xmlns=\"http://www.topografix.com/GPX/1/1\" "));
    file.print(F("xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" "));
    file.print(F("xmlns:gpxtpx=\"http://www.garmin.com/xmlschemas/TrackPointExtension/v2\" "));
    file.print(F("xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 "));
    file.print(F("http://www.topografix.com/GPX/1/1/gpx.xsd "));
    file.print(F("http://www.garmin.com/xmlschemas/TrackPointExtension/v2 "));
    file.print(F("https://www8.garmin.com/xmlschemas/TrackPointExtensionv2.xsd\">\n"));
    file.print(F("<trk><trkseg>\n"));
}

void writeTrackPoint(File &file, double lat, double lon, double speed, float heading, double timestamp) {
    file.print("<trkpt lat=\"");
    file.print(lat, 6);
    file.print("\" lon=\"");
    file.print(lon, 6);
    file.println("\">");
    file.print("<time>");
    file.print(timestamp, 2);
    file.println("</time>");
    file.println("<extensions>");
    file.println("<gpxtpx:TrackPointExtension>");
    file.print("<gpxtpx:speed>");
    file.print(speed, 2);
    file.println("</gpxtpx:speed>");
    file.print("<gpxtpx:heading>");
    file.print(heading, 2);
    file.println("</gpxtpx:heading>");
    file.println("</gpxtpx:TrackPointExtension>");
    file.println("</extensions>");
    file.println("</trkpt>");
}

void closeGPXFile(File &file) {
    log::toAll("closing GPX file; copying waypoints");
    file.println("</trkseg></trk>");
    // now copy all of the waypoints to this file
    if (WPTlogFile) {
      WPTlogFile.close();
      WPTlogFile = SPIFFS.open("/wpt.log", "r", false);
      while (WPTlogFile.available()) {
        file.write(WPTlogFile.read());
      }
      WPTlogFile.close();
      WPTlogFile = SPIFFS.open("/wpt.log", "w", true);
      if (!WPTlogFile)
        log::toAll("failed to RE-open WPT log");
    }
    file.println("</gpx>");
    file.close();
}

void writeHeadingWaypoint(File &file, double lat, double lon, double speed, float heading, double timestamp) {
  // using heading (heading) as <name>
  file.printf("<wpt lat=\"%.6f\" lon=\"%.6f\">\n", lat, lon);
  file.printf("<time>%0.2f</time>\n", timestamp);
  file.printf("<name>H:%2.2f</name>\n", heading);
  file.print("</wpt>\n");
}

void initGPX() {
    sprintf(logTempName, "/%s%d.log",GPXlog,logFileIdx);
    if (SPIFFS.exists(logTempName)) {
        GPXlogFile = SPIFFS.open(logTempName, "a", false);
    } else {
        GPXlogFile = SPIFFS.open(logTempName, "w", true);
        writeGPXHeader(GPXlogFile);
    }
    if (!GPXlogFile)
        log::toAll("failed to open GPX log");
    WPTlogFile = SPIFFS.open("/wpt.log", "w", true);
    if (!WPTlogFile)
        log::toAll("failed to open WPT log");
}

void startNextLog() {
  log::toAll("starting next GPX logfile @" + String(logFileIdx+1) + " time: " + String(millis()));
  // close .gpx file and start a new one
  if (GPXlogFile) {
    ++logFileIdx;
    closeGPXFile(GPXlogFile);
    sprintf(logTempName, "%s%5d.log",GPXlog,logFileIdx);
    preferences.putInt("logFileIdx",logFileIdx);
    GPXlogFile = SPIFFS.open(logTempName, "w", true);
    writeGPXHeader(GPXlogFile);
  } else {
    log::toAll("invalid GPX log file, index: " + logFileIdx);
  }
}
#endif