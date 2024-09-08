/* having gotten ESPnow working, I think I'm going to retire all this code and just send 
   PGN for heading on the bus, since I can connect the compass to the wind bus and correct inline
   At the same time, I probably need to retain ESPnow for control commands to the compass (such as orientation)
   so I might keep that part as-is
   ESPnow requires me (currently) to hard code the MAC addresses of client and server
   REMEMBER TO CHANGE THE MAC ADDRESSES IN THE CODE
*/
#ifdef ESPNOW
#include <esp_now.h>
#include <WiFi.h>
#include "ESPAsyncWebServer.h"
#include <Arduino_JSON.h>
#include <Wire.h>

#include "compass.h"

#ifdef BNO08X
#include <Adafruit_BNO08x.h>
extern sh2_SensorValue_t sensorValue;
#endif

//extern static int variation;
extern float mastCompassDeg; 
extern float boatCompassDeg;
extern int boatCompassCalStatus;
extern float mastDelta;
// how is the compass oriented on the board relative to boat compass
// when mast is centered, mast compass+orientation == boat compass
extern int mastOrientation;
extern int mastFrequency;
extern bool compassOnToggle;
extern JSONVar readings;

uint8_t compassAddress[ESP_NOW_ETH_ALEN];  // = {0xE4, 0x65, 0xB8, 0x78, 0xE9, 0x7C};
//uint8_t compassAddress[] = {0x08, 0xB6, 0x1F, 0xB8, 0x66, 0x3C};
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t peerInfo;
bool foundPeer = false;
int rxCount;

compass_s outCommand;

extern bool compassOnToggle;

extern AsyncWebServer server;
extern AsyncEventSource events;
extern int mastAngle[];

float getCompass(int correction);
float calculateHeading(float r, float i, float j, float k, int correction);

sh2_SensorValue_t sensorValueMast;

// callback function that will be executed when data is received from mast compass
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) { 
    rxCount++;
//#define DEBUG
#ifdef DEBUG
    // Copies the sender mac address to a string
    char macStr[64];
    Serial.print("Packet received from: ");
    snprintf(macStr, sizeof(macStr), "0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x",
            mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    Serial.println(macStr);
#endif
    if (foundPeer) { // we have a peer so process compass data
        memcpy(&sensorValueMast, incomingData, sizeof(sensorValueMast));
        //Serial.printf("sensorValueMast: %d\n", sensorValueMast.sensorId);
        if (sensorValueMast.sensorId == SH2_ROTATION_VECTOR) {
            //Serial.printf(">3D|quaternionSphere:%0.4ld:S:sphere:P:0:0:0:Q:%.3f:%.3f:%.3f:%.3f:RA:1:C:blue\n",millis(),sensorValueMast.un.rotationVector.real, sensorValueMast.un.rotationVector.i, sensorValueMast.un.rotationVector.j, sensorValueMast.un.rotationVector.k);
            //Serial.printf(" mast accuracy: %d ", sensorValueMast.un.rotationVector.accuracy);
            //Serial.printf(" mast cal status: %d\n", sensorValueMast.status);
            unsigned long ts = millis();
            mastCompassDeg = calculateHeading(sensorValueMast.un.rotationVector.real, sensorValueMast.un.rotationVector.i, sensorValueMast.un.rotationVector.j, sensorValueMast.un.rotationVector.k, 0);
            Serial.printf(">Mr:%0.4ld:%0.2f\n", ts, sensorValueMast.un.rotationVector.real);
            Serial.printf(">Mi:%0.4ld:%0.2f\n", ts, sensorValueMast.un.rotationVector.i); 
            Serial.printf(">Mj:%0.4ld:%0.2f\n", ts, sensorValueMast.un.rotationVector.j);
            Serial.printf(">Mk:%0.4ld:%0.2f\n", ts, sensorValueMast.un.rotationVector.k); 
            Serial.printf(">Mhead:%0.4ld:%0.0f\n", ts, mastCompassDeg);
            ts = millis();
            //boatCompassDeg = calculateHeading(sensorValue.un.rotationVector.real, sensorValue.un.rotationVector.i, sensorValue.un.rotationVector.j, sensorValue.un.rotationVector.k, 0);        
            Serial.printf(">Br:%0.4ld:%0.2f\n", ts, sensorValue.un.rotationVector.real);
            Serial.printf(">Bi:%0.4ld:%0.2f\n", ts, sensorValue.un.rotationVector.i);
            Serial.printf(">Bj:%0.4ld:%0.2f\n", ts, sensorValue.un.rotationVector.j);
            Serial.printf(">Bk:%0.4ld:%0.2f\n", ts, sensorValue.un.rotationVector.k);   
            Serial.printf(">Bhead:%0.4ld:%0.0f\n", ts, boatCompassDeg);
        } else
            Serial.printf("unknown sensorID %d\n", sensorValueMast.sensorId);
    } else { // first time hearing from peer
        Serial.printf("found peer\n");
        foundPeer = true;
        //Serial.printf("\nchannel: %d ifidx: %x encrypt: %d addr size %d (6)\n", peerInfo.channel, peerInfo.ifidx, peerInfo.encrypt, sizeof(peerInfo.peer_addr));
        Serial.printf("ESP peer MAC addr: ");
        for (int i=0; i<ESP_NOW_ETH_ALEN; i++)
            Serial.printf("%02X ", mac_addr[i]);
        }
        //memcpy(compassAddress, peerInfo.peer_addr, ESP_NOW_ETH_ALEN);  // set compass address
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
// TBD: retry if no answer to broadcast
void setupESPNOW() {
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    } else
        Serial.println("ESP-NOW Initialized");
    
    // Once ESPNow is successfully init, register callbacks
    esp_now_register_recv_cb(OnDataRecv);
    esp_now_register_send_cb(OnDataSent);
}

void espNowBroadcast() {
    // broadcast to see who's out there
    memcpy(peerInfo.peer_addr, broadcastAddress, ESP_NOW_ETH_ALEN);    // memcpy(dest,src)
    peerInfo.encrypt = false;
    peerInfo.channel = 0;
    int err;  
    if (err=esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.printf("Failed to add broadcast peer: %d\n", err);
        return;
    } else Serial.println("ESP-NOW broadcast peer added");
    esp_err_t result;
    for (int retries = 0; retries < 5; retries++) {
        result = esp_now_send(broadcastAddress, (uint8_t *) &outCommand, sizeof(outCommand));
        if (result == ESP_OK) {
            Serial.printf("sent broadcast\n");
        } else {
            Serial.printf("Error sending the data: %d\n", result);
        }
        delay(100);
    }
}
#endif