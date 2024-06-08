/* having gotten ESPnow working, I think I'm going to retire all this code and just send 
   PGN for heading on the bus, since I can connect the compass to the wind bus and correct inline
   At the same time, I probably need to retain ESPnow for control commands to the compass (such as orientation)
   so I might keep that part as-is
   ESPnow requires me (currently) to hard code the MAC addresses of client and server
   REMEMBER TO CHANGE THE MAC ADDRESSES IN THE CODE
*/

#include <esp_now.h>
#include <WiFi.h>
#include "ESPAsyncWebServer.h"
#include <Arduino_JSON.h>
#include <Wire.h>

// defs for robotshop CMPS14
extern int CMPS14_ADDRESS;  // Address of CMPS14 shifted right one bit for arduino wire library
extern bool cmps14_ready;
#define ANGLE_8  1           // Register to read 8bit angle from

unsigned char high_byte, low_byte, angle8;
char pitch, roll;
unsigned int angle16, comp8, comp16;
#define VARIATION -15.2
static int variation;
float mastCompassDeg; 
float boatCompassDeg;
// how is the compass oriented on the board relative to boat compass
// when mast is centered, mast compass+orientation == boat compass
int mastOrientation;
extern int mastFrequency;
extern bool compassOnToggle;
extern JSONVar readings;

uint8_t compassAddress[] = {0xE4, 0x65, 0xB8, 0x78, 0xE9, 0x7C};
esp_now_peer_info_t peerInfo;

// struct we will send to compass
typedef struct control_s {
  bool compassOnToggle;
  int orientation;
  int frequency;
} control_s;
control_s outCommand;

extern bool compassOnToggle;

JSONVar board;

extern AsyncWebServer server;
extern AsyncEventSource events;
extern int mastAngle[];

float getCompass(int correction);

// callback function that will be executed when data is received from mast compass
// we will also check/update boat heading from local compass at this time
// not using at this time
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) { 
}

// Send control message via ESP-NOW
// called from webserver.cpp when settings are updated "get /params"
bool sendMastControl() {
  outCommand.compassOnToggle = compassOnToggle;
  outCommand.orientation = mastOrientation;
  outCommand.frequency = mastFrequency;
  esp_err_t result = esp_now_send(compassAddress, (uint8_t *) &outCommand, sizeof(outCommand));
  if (result == ESP_OK) {
    Serial.printf("sent mast control\n");
    return true;
  } else {
    Serial.printf("Error sending the data: %d\n", result);
  }
  return false;
}

// Callback when data is sent
String success;
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status:\t");
  Serial.print(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success: " : "Delivery Fail: ");
  Serial.println(status);
}

void compassCommand() {
  outCommand.compassOnToggle = compassOnToggle;
  outCommand.orientation = 0;
  esp_err_t result = esp_now_send(compassAddress, (uint8_t *) &outCommand, sizeof(outCommand));
  Serial.printf("compassCommand err %d\n", result);
}

// Init ESP-NOW
void setupESPNOW() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  } else
    Serial.println("ESP-NOW Initialized");
  
  // Once ESPNow is successfully init, register callbacks
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, compassAddress, 6);
  peerInfo.encrypt = false;
  peerInfo.channel = 0;

  Serial.printf("ESP peer MAC addr: ");
  for (int i=0; i<ESP_NOW_ETH_ALEN; i++)
    Serial.printf("%02X ", peerInfo.peer_addr[i]);
  Serial.printf("\nchannel: %d ifidx: %x encrypt: %d\n", peerInfo.channel, peerInfo.ifidx, peerInfo.encrypt);
  // Add peer      
  int err;  
  if (err=esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.printf("Failed to add peer: %d\n", err);
    return;
  } else Serial.println("ESP-NOW peer added");
}

// get heading from (local) compass
// called when we get a Heading PGN on the wind bus (which means mast compass transmitted)
// so frequency of update is going to depend on how often we get a Heading PGN from the mast
// also called as a Reaction in case we're not connected to mast compass
// TBD: read calibration status
float getCompass(int correction) {
  //Serial.println("getCompass");
  if (cmps14_ready) {
    Wire.beginTransmission(CMPS14_ADDRESS);  // starts communication with CMPS14
    Wire.write(ANGLE_8);                     // Sends the register we wish to start reading from
    Wire.endTransmission();
    //if (!error) { // we found CMPS14
    // Request 5 bytes from the CMPS12
    // this will give us the 8 bit bearing, 
    // both bytes of the 16 bit bearing, pitch and roll
    Wire.requestFrom(CMPS14_ADDRESS, 5); 
    while((Wire.available() < 5)); // (this can hang)
/*
    int cycles = 0;
    int availbytes = 0;
    while (availbytes < 5 && cycles++ < 1000) {
      availbytes += Wire.available();
    }
    // Wait for all bytes to come back, hangs if no CMPS14 attached  
    while((Wire.available() < 5) && cycles++ < 100) delay(10);
    if (cycles == 100) {
      Serial.printf("timeout on CMPS14 read\n");
      return -1.0;
    } else 
*/
    angle8 = Wire.read();               // Read back the 5 bytes
    comp8 = map(angle8, 0, 255, 0, 359);
    comp8 = (comp8 + correction + 360) % 360;
    high_byte = Wire.read();
    low_byte = Wire.read();
    pitch = Wire.read();
    roll = Wire.read();
    
    angle16 = high_byte;                 // Calculate 16 bit angle
    angle16 <<= 8;
    angle16 += low_byte;
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
    return (float)comp16;
  }
  return -1.0;
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
