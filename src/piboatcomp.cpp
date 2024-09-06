#ifdef PICOMPASS
#include <Arduino.h>
#include <HardwareSerial.h>

HardwareSerial SerialPort(2); // Use UART2

#define RX_PIN 34
//#define TX_PIN 25   // do I need TX?

extern float boatCompassPi;

void setupPiComp() {
  //Serial.begin(115200); delay(500);
  Serial.println("Starting Serial Communication");
  SerialPort.begin(38400, SERIAL_8N1, RX_PIN, -1);
}

int piCompCount=0;

void loopPiComp() {
  char incomingChar;
  if (SerialPort.available()) {
    String data = SerialPort.readStringUntil('\n');
    
    int headingPos = data.indexOf("heading");
    if (headingPos != -1) {
      // Extract the substring starting from "heading"
      String headingStr = data.substring(headingPos);
      // Find the space after "heading"
      int spacePos = headingStr.indexOf(' ');
      if (spacePos != -1) {
        // Extract the number after "heading"
        String headingValue = headingStr.substring(spacePos + 1);
        // Convert the string to a float
        boatCompassPi = headingValue.toFloat();
        //Serial.print("Extracted heading: ");
        //Serial.println(boatCompassPi, 2);
        piCompCount++;
      }
    }
  }
}
#endif