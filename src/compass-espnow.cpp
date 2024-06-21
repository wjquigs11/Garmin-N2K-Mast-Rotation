/* having gotten ESPnow working, I think I'm going to retire all this code and just send 
   PGN for heading on the bus, since I can connect the compass to the wind bus and correct inline
   At the same time, I probably need to retain ESPnow for control commands to the compass (such as orientation)
   so I might keep that part as-is
   ESPnow requires me (currently) to hard code the MAC addresses of client and server
   REMEMBER TO CHANGE THE MAC ADDRESSES IN THE CODE
*/
#include <Arduino.h>
#include <ActisenseReader.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <N2kMessages.h>
#include <NMEA2000_esp32.h>
#include <NMEA0183Msg.h>
#include <NMEA0183Messages.h>
#include "NMEA0183Handlers.h"
#include "BoatData.h"
#include <ReactESP.h>
#include <Wire.h>
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>
#include <movingAvg.h>
#include "elapsedMillis.h"
#include <Arduino.h>
#include <N2kMessages.h>
#include <Adafruit_ADS1X15.h>
#include <WiFi.h>
#include "SPIFFS.h"
#include <Arduino_JSON.h>
#include <ESPmDNS.h>
#include "mcp2515.h"
#include "can.h"
#include "Async_ConfigOnDoubleReset_Multi.h"
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include <WebSerial.h>
#include <Adafruit_BNO08x.h>

#include <esp_now.h>
#include "compass.h"
#include "windparse.h"

// defs for robotshop CMPS14
//extern int CMPS14_ADDRESS;  // Address of CMPS14 shifted right one bit for arduino wire library
#define ANGLE_8  1          // Register to read 8bit angle from (starting...we read 5 bytes)
#define CAL_STATE 0x1E      // register to read calibration state

unsigned char high_byte, low_byte, angle8;
char pitch, roll;
unsigned int angle16, comp8, comp16;
#define VARIATION -15.2
static int variation;
float mastCompassDeg; 
float boatCompassDeg;
int boatCompassCalStatus;
float mastDelta;
// how is the compass oriented on the board relative to boat compass
// when mast is centered, mast compass+orientation == boat compass
int mastOrientation;
extern int mastFrequency;
extern bool compassOnToggle;
extern JSONVar readings;

// peer (compass) MAC address
//uint8_t compassAddress[] = {0xC0, 0x49, 0xEF, 0xAB, 0x5E, 0x38};
//uint8_t serverAddress[] = {0x08, 0xB6, 0x1F, 0xB8, 0xC4, 0xAC};
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t compassAddress[ESP_NOW_ETH_ALEN];
esp_now_peer_info_t peerInfo;
#define MAX_CHANNEL 11  // 11 in North America or 13 in Europe
int32_t channel;
bool compassPeered = false;

control_s outCommand;

extern bool compassOnToggle;

JSONVar board;

extern AsyncWebServer server;
extern AsyncEventSource events;
extern int mastAngle[];

float getCompass(int correction);
void logToAll(String message);
void logToAlln(String message);
bool sendMastControl();

#define SBUF 64
char strBuf[SBUF];

// callback function that will be executed when data is received from mast compass
// first will be compass broadcasting on different channels to peer
// after peering, packet is an ack to command and will contain compass hostname
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) { 
  logToAlln("OnDataRecv");
  Serial.printf("%d bytes received from: ", len);
  // Copies the sender mac address to a string
  sprintf(strBuf, "{ 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x }",
          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  //logToAll(String(strBuf) + "\ndata: ");
  Serial.println(strBuf);
  if (!compassPeered) {
    Serial.printf("pairing request on channel %d\n", channel);
    for (int i=0; i<ESP_NOW_ETH_ALEN; i++) compassAddress[i] = mac_addr[i];
    // Add peer 
    memcpy(peerInfo.peer_addr, compassAddress, ESP_NOW_ETH_ALEN);
    peerInfo.encrypt = false;
    peerInfo.channel = channel;     
    int err;
    if ((err=esp_now_add_peer(&peerInfo)) != ESP_OK){
      Serial.printf("Failed to add peer: %d\n", err);
      return;
    } else Serial.println("ESP-NOW peer added");
    if (sendMastControl()) {
      Serial.println("SendMastControl success");
      // compass now knows it can use this channel
      compassPeered = true;
    } else {
      Serial.println("SendMastControl failed");
    }
  } else {
    // compass peered
    // might be a good idea to send heading with ACK to compare to N2K reading
    Serial.print("compass data: ");
    for (int i=0; i<len; i++) {
      //if (i==SBUF) break;
      Serial.printf("%c", incomingData[i]);
    }
    Serial.println();
  }
}

int retries=0;
#define MAX_TRY 5
bool ESPNOWsuccess=false;

// Send control message via ESP-NOW
// called from webserver.cpp when settings are updated "get /params"
bool sendMastControl() {
  bool retval = false;
  logToAlln("sendMastControl");
#if defined(MASTCOMPASS)
  outCommand.compassOnToggle = compassOnToggle;
  outCommand.orientation = mastOrientation;
  outCommand.frequency = mastFrequency;
  esp_err_t result = esp_now_send(compassAddress, (uint8_t *) &outCommand, sizeof(outCommand));
  Serial.printf("sendMastControl sending %d bytes\n", sizeof(outCommand));
  if (result == ESP_OK) {
    Serial.println("sent mast control");
    retval = true;
  } else {
    Serial.println("Error sending the data");
    retval = false;
  }
#endif
  return retval;
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  logToAlln("OnDataSent");
  Serial.print("Last Packet Send Status:\t");
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("Delivery Success");
    retries = 0;
  } else {
    Serial.printf("Delivery Fail: %d\n", status);
    retries++;
    if (retries < MAX_TRY) {
    // retry 3x until we have compassHost
      esp_err_t result = esp_now_send(compassAddress, (uint8_t *) &outCommand, sizeof(outCommand));
      Serial.printf("retry %d sendMastControl sending %d bytes\n", retries, sizeof(outCommand));
      if (result == ESP_OK) Serial.println("sent mast control");
        else Serial.println("Error sending the data");
    } else {
      retries = 0;
      Serial.println("Max retries on ESPNOW.");
      // send controls to mast compass using HTTP instead
    }
  }
}

void compassCommand() {
  outCommand.compassOnToggle = compassOnToggle;
  outCommand.orientation = 0;
  esp_err_t result = esp_now_send(compassAddress, (uint8_t *) &outCommand, sizeof(outCommand));
  //Serial.printf("compassCommand err %d\n", result);
  //logToAlln("compassCommand error" + String(result));
}

// Init ESP-NOW
void setupESPNOW() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  } else Serial.println("ESP-NOW Initialized");

  channel = WiFi.channel();
  //Serial.printf("ESP wifi channel: " + String(channel) + "\n");
  Serial.printf("ESP wifi channel: %d\n", channel);

  // Once ESPNow is successfully init, register callbacks
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
}


// get heading from (local) compass
// called when we get a Heading PGN on the wind bus (which means mast compass transmitted)
// so frequency of update is going to depend on how often we get a Heading PGN from the mast
// also called as a Reaction in case we're not connected to mast compass
// TBD: make this object oriented and overload getCompass for either CMPS14 or BNO085
float getCMPS14(int correction) {
  //Serial.println("getCompass");
    Wire.beginTransmission(CMPS14_ADDRESS);  // starts communication with CMPS14
    Wire.write(ANGLE_8);                     // Sends the register we wish to start reading from
    Wire.endTransmission();
    // Request 5 bytes from the CMPS12
    // this will give us the 8 bit bearing, 
    // both bytes of the 16 bit bearing, pitch and roll
    Wire.requestFrom(CMPS14_ADDRESS, 5); 
    while((Wire.available() < 5)); // (this can hang?)
    angle8 = Wire.read();               // Read back the 5 bytes
    comp8 = map(angle8, 0, 255, 0, 359);
    comp8 = (comp8 + correction + 360) % 360;
    high_byte = Wire.read();
    low_byte = Wire.read();
    pitch = Wire.read();
    roll = Wire.read();
    // TBD set up PGN for attitude and transmit
    angle16 = high_byte;                 // Calculate 16 bit angle
    angle16 <<= 8;
    angle16 += low_byte;
    // doesn't this drop to an int?
    comp16 = ((angle16/10) + correction + 360) % 360;
//#define DEBUG
#ifdef DEBUG
    Serial.print("roll: ");               // Display roll data
    Serial.print(roll, DEC);
    
    Serial.print("    pitch: ");          // Display pitch data
    Serial.print(pitch, DEC);
    
    Serial.print("    angle full: ");     // Display 16 bit angle with decimal place
    Serial.print(angle16 / 10, DEC);
    Serial.print(".");
    Serial.print(angle16 % 10, DEC);

    Serial.print("    comp16: ");
    Serial.print(comp16, DEC);
    
    Serial.print("     comp8: ");        // Display 8bit angle
    Serial.println(comp8, DEC);
#endif
    Wire.beginTransmission(CMPS14_ADDRESS);  // starts communication with CMPS14
    Wire.write(CAL_STATE);                     // Sends the register we wish to start reading from
    Wire.endTransmission();
    Wire.requestFrom(CMPS14_ADDRESS, 5); 
    while((Wire.available() < 1)); // (this can hang)
    boatCompassCalStatus = Wire.read() & 0x3;
    //Serial.printf("calStatus: 0x%x\n", boatCompassCalStatus);
    return (float)comp16;
}

float calculateHeading(float r, float i, float j, float k, int correction);

// TBD make this a library shared between controller and mast compass
float getBNO085(int correction) {
  float accuracy, heading;
  // not checking timing here since it's controlled by ReactESP
  //unsigned long currentMillis = millis();
  //if (currentMillis - previousReading < BNOREADRATE) {
  //  logToAll("reading too soon" + String(currentMillis) + "-" + String(previousReading) + "\n");
  //  return -2.0; // minimum delay in case displayDelay is set too low
  //}
  //previousReading = currentMillis;

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 100))
      Serial.println("Could not enable rotation vector");
    else
      Serial.println("enabled abs rotation vector");
  }
  if (!bno08x.getSensorEvent(&sensorValue)) {
    return -3.0;
  }
  // printf("ID: %d\n", sensorValue.sensorId);
  /* Status of a sensor
   *   0 - Unreliable
   *   1 - Accuracy low
   *   2 - Accuracy medium
   *   3 - Accuracy high
   */
  boatCompassCalStatus = sensorValue.status;
  switch (sensorValue.sensorId) {
  case SH2_ROTATION_VECTOR:
    accuracy = sensorValue.un.rotationVector.accuracy;
    heading = calculateHeading(sensorValue.un.rotationVector.real, sensorValue.un.rotationVector.i, sensorValue.un.rotationVector.j, sensorValue.un.rotationVector.k, correction);      //logToAll("heading1: " + String(heading) + "  cal: " + calStatus + "\n");
    return heading;
    break;
  default:
    printf("ID: %d\n", sensorValue.sensorId);
    break;
  }
  return -4.0;
}

// Function to calculate tilt-compensated heading from a quaternion
float calculateHeading(float r, float i, float j, float k, int correction) {
  // Convert quaternion to rotation matrix
  float r11 = 1 - 2 * (j * j + k * k);
  // change r21 to =-2 if sensor is upside down
  float r21 = 2 * (i * j + r * k);
  float r31 = 2 * (i * k - r * j);
  float r32 = 2 * (j * k + r * i);
  float r33 = 1 - 2 * (i * i + j * j);

  // Calculate pitch (theta) and roll (phi)
  float theta = -asin(r31);
  float phi = atan2(r32, r33);
  // Calculate yaw (psi)
  float psi = atan2(r21, r11);
  float heading = (psi * 180 / M_PI) + correction;
  // correction may be positive or negative
  if (heading > 360) heading -= 360;
  if (heading < 0) heading += 360;
  return heading;
}

float getCompass(int correction) {
  if (!compassReady)
    return -1;
  if (compassType == CMPS14)
    return getCMPS14(correction);
  else if (compassType == BNO085)
    return getBNO085(correction);
  else
    return -2;
}

/*
https://www.arduino.cc/reference/en/language/functions/communication/wire/endtransmission/
endTransmission() returns:
0: success.
1: data too long to fit in transmit buffer.
2: received NACK on transmit of address.
3: received NACK on transmit of data.
4: other error.
5: timeout
*/
