#ifdef WEBSERIAL
#include "include.h"

bool debugFlag = false;

#if 0
void i2cScan(TwoWire Wire) {
  byte error, address;
  int nDevices = 0;

  log::toAll("Scanning...");

  for (address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmission to see if
    // a device acknowledged the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    sprintf(prbuf, "%2X", address); // Formats value as uppercase hex

    if (error == 0) {
      log::toAll("I2C device found at address 0x" + String(prbuf));
      nDevices++;
    }
    else if (error == 4) {
      log::toAll("error at address 0x" + String(prbuf));
    }
  }

  if (nDevices == 0) {
    log::toAll("No I2C devices found\n");
  } else {
    log::toAll("done\n");
  }
}
#endif

String formatMacAddress(const String& macAddress) {
  String result = "{";
  int len = macAddress.length();
  
  for (int i = 0; i < len; i += 3) {
    if (i > 0) {
      result += ", ";
    }
    result += "0x" + macAddress.substring(i, i + 2);
  }
  
  result += "};";
  return result;
}

String commandList[] = {"restart", "hostname", "status", "wifi", "conslog", "debug", "spiffs"};
#define ASIZE(arr) (sizeof(arr) / sizeof(arr[0]))
String words[10]; // Assuming a maximum of 10 words

void WebSerialonMessage(uint8_t *data, size_t len) {
  Serial.printf("Received %lu bytes from WebSerial: ", len);
  Serial.write(data, len);
  Serial.println();
  //WebSerial.print("Received: ");
  String dataS = String((char*)data);
  // Split the String into an array of Strings using spaces as delimiters
  String words[10]; // Assuming a maximum of 10 words
  int wordCount = 0;
  int startIndex = 0;
  int endIndex = 0;
  while (endIndex != -1) {
    endIndex = dataS.indexOf(' ', startIndex);
    if (endIndex == -1) {
      words[wordCount++] = dataS.substring(startIndex);
    } else {
      words[wordCount++] = dataS.substring(startIndex, endIndex);
      startIndex = endIndex + 1;
    }
  }
  for (int i = 0; i < wordCount; i++) {
    int j;
    WebSerial.print(words[i]);
    if (words[i].equals("?")) {
      for (j = 1; j < ASIZE(commandList); j++) {
        WebSerial.println(String(j) + ":" + commandList[j]);
      }
      return;
    }
    if (words[i].equals("restart")) {
      WebSerial.println("restarting...");
      ESP.restart();
    }
    if (words[i].equals("scan")) {
      //i2cScan();
      return;
    }
    if (words[i].startsWith("host")) {
      if (!words[++i].isEmpty()) {
        host = words[i];
        preferences.putString("hostname", host);
        log::toAll("hostname set to " + host);
        log::toAll("restart to change hostname");
        log::toAll("preferences " + preferences.getString("hostname"));
      } else {
        log::toAll("hostname: " + host);
      }
      return;
    }
    if (words[i].equals("status")) {
      time_t ts = lastUpdate/1000;
      ptm = localtime(&ts);
      sprintf(prbuf,"[%02d/%02d %02d:%02d:%02d] ",ptm->tm_mon+1,ptm->tm_mday,ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
      String buf = String(prbuf); 
      unsigned long uptime = millis() / 1000;
      buf += "uptime: " + String(uptime);
      log::toAll(buf);
      buf = String();
      log::toAll(getSensorReadings());
      return;
    }
    if (words[i].startsWith("wifi")) {
      String buf = "hostname: " + host;
      buf += " wifi: " + WiFi.SSID();
      buf += " ip: " + WiFi.localIP().toString();
      buf += "  MAC addr: " + formatMacAddress(WiFi.macAddress());
      log::toAll(buf);
      // TBD: start captive portal?
      buf = String();
      return;
    }
    // 'spiffs ls'
    // 'spiffs status'
    // 'spiffs format'
    if (words[i].startsWith("spiffs")) {
      if (wordCount > 1) {
        if (words[++i].equals("ls")) {
          File root = SPIFFS.open("/");
          File file = root.openNextFile();
          while (file) {
            WebSerial.println(file.name());
            file.close(); // Close the file after reading its name
            file = root.openNextFile();
          }
          root.close();
          WebSerial.println("done");
          return;
        }
        if (words[i].startsWith("status")) {
          size_t totalBytes = SPIFFS.totalBytes();
          size_t usedBytes = SPIFFS.usedBytes();
          float usedPercentage = ((float)usedBytes / totalBytes) * 100;
          WebSerial.print("SPIFFS Total space: ");
          WebSerial.print(totalBytes);
          WebSerial.println(" bytes");
          WebSerial.print("Used space: ");
          WebSerial.print(usedBytes);
          WebSerial.println(" bytes");
          WebSerial.print("Free space: ");
          WebSerial.print(totalBytes - usedBytes);
          WebSerial.println(" bytes");
          WebSerial.print("Usage: ");
          WebSerial.print(usedPercentage);
          WebSerial.println("%");
          return;
        }
        if (words[i].equals("read")) {
          if (wordCount > 2) {
            File file = SPIFFS.open(words[++i]);
            if (!file || file.isDirectory()) {
              log::toAll("Failed to open wifi.txt for reading");
              return;
            }
            String fileContent = "";
            while (file.available()) {
              fileContent = file.readStringUntil('\n');
              WebSerial.println(fileContent);
            }
            file.close();
          }
          return;
        }
        if (words[i].equals("format")) {
          SPIFFS.format();
          WebSerial.println("SPIFFS formatted");
          return;
        }
      }
      WebSerial.println("spiffs {ls|status|format}");
      return;
    } // spiffs
    if (words[i].startsWith("log")) {
      log::logToSerial = !log::logToSerial;
      log::toAll("serial log: " + String(log::logToSerial ? "on" : "off"));
      return;
    }
    if (words[i].startsWith("debug")) {
      debugFlag = !debugFlag;
      log::toAll("debug: " + String(debugFlag ? "on" : "off"));
      return;
    }
    // 'conslog reset' overwrites console log and restarts
    // 'conslog' (no arguments) displays existing log
    if (words[i].startsWith("conslog")) {
      if (wordCount > 1 && words[++i].startsWith("reset")) {
        consLog.close();
        consLog = SPIFFS.open("/console.log", "w", true);
        if (!consLog) {
          log::toAll("failed to open console log");
        }
        log::toAll("restarted console log");
        return;
      } else {
        // Print the last 20 lines of the console log file
        File logFile = SPIFFS.open("/console.log", "r");
        if (!logFile) {
          log::toAll("Failed to open console log file");
          return;
        }
        // Store the last 20 lines in a circular buffer
        const int maxLines = 20;
        String lineBuffer[maxLines];
        int lineCount = 0;
        int bufferIndex = 0;
        // Read all lines, keeping only the last 20
        while (logFile.available()) {
          String line = logFile.readStringUntil('\n');
          lineBuffer[bufferIndex] = line;
          bufferIndex = (bufferIndex + 1) % maxLines;
          if (lineCount < maxLines) lineCount++;
        }
        // Print the lines in the correct order
        for (int i = 0; i < lineCount; i++) {
          int index = (bufferIndex + i) % maxLines;
          WebSerial.print(lineBuffer[index]);
        }
        logFile.close();
        return;
      }
    }
    log::toAll("Unknown command: " + words[i]);
  }
  for (int i=0; i<wordCount; i++) words[i] = String();
  dataS = String();
}
#endif
