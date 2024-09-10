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

#include "windparse.h"
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
extern float boatCalStatus;
// how is the compass oriented on the board relative to boat compass
// when mast is centered, mast compass+orientation == boat compass
extern int mastOrientation;
extern int mastFrequency;
extern bool compassOnToggle;
extern JSONVar readings;
extern bool teleplot=false;

uint8_t compassAddress[ESP_NOW_ETH_ALEN];  // = {0xE4, 0x65, 0xB8, 0x78, 0xE9, 0x7C};
//uint8_t compassAddress[] = {0x08, 0xB6, 0x1F, 0xB8, 0x66, 0x3C};
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t peerInfo;
bool foundPeer = false;
int rxCount;
extern int reportType;

compass_s outCommand;

extern bool compassOnToggle;

extern AsyncWebServer server;
extern AsyncEventSource events;
extern int mastAngle[];

float getCompass(int correction);
float calculateHeading(float r, float i, float j, float k, int correction);
float readCompassDelta();

sh2_SensorValue_t sensorValueMast;

void logToAll(String s);

float calculateYawDifference(float w1, float i1, float j1, float k1,
                             float w2, float i2, float j2, float k2) {
  // Step 1: Calculate the difference quaternion
  float w = w1*w2 + i1*i2 + j1*j2 + k1*k2;
  float i = w1*i2 - i1*w2 - j1*k2 + k1*j2;
  float j = w1*j2 + i1*k2 - j1*w2 - k1*i2;
  float k = w1*k2 - i1*j2 + j1*i2 - k1*w2;
  
  // Step 2: Normalize the difference quaternion
  float magnitude = sqrt(w*w + i*i + j*j + k*k);
  w /= magnitude;
  i /= magnitude;
  j /= magnitude;
  k /= magnitude;
  
  // Step 3: Convert the difference quaternion to Euler angles
  float yaw = atan2(2.0f * (w*k + i*j), 1.0f - 2.0f * (j*j + k*k));
  
  // Step 4: Convert yaw from radians to degrees
  yaw = yaw * 180.0f / PI;
  
  // Step 5: Ensure the result is in the range [-180, 180]
  if (yaw > 180.0f) yaw -= 360.0f;
  if (yaw < -180.0f) yaw += 360.0f;
  
  return yaw;
}

void teleplotter(unsigned long ts, float Mreal, float Mi, float Mj, float Mk, float Breal, float Bi, float Bj, float Bk) {
    Serial.printf(">Mr:%0.4ld:%0.2f\n", ts, Mreal);
    Serial.printf(">Mi:%0.4ld:%0.2f\n", ts, Mi); 
    Serial.printf(">Mj:%0.4ld:%0.2f\n", ts, Mj);
    Serial.printf(">Mk:%0.4ld:%0.2f\n", ts, Mk); 
    Serial.printf(">Mhead:%0.4ld:%0.0f\n", ts, mastCompassDeg);
    ts = millis();
    Serial.printf(">Br:%0.4ld:%0.2f\n", ts, Breal);
    Serial.printf(">Bi:%0.4ld:%0.2f\n", ts, Bi);
    Serial.printf(">Bj:%0.4ld:%0.2f\n", ts, Bj);
    Serial.printf(">Bk:%0.4ld:%0.2f\n", ts, Bk);   
    Serial.printf(">Bhead:%0.4ld:%0.0f\n", ts, boatCompassDeg);
    float rotateM = calculateYawDifference(Mreal, Mi, Mj, Mk, Breal, Bi, Bj, Bk);
    Serial.printf(">Mrotate:%0.4ld:%0.0f\n", ts, rotateM);
}

void teleplotterNAC(unsigned long ts, sh2_RotationVector_t Mast, sh2_RotationVector_t Boat) {
    teleplotter(ts, Mast.real, Mast.i, Mast.j, Mast.k, Boat.real, Boat.i, Boat.j, Boat.k);
}

void teleplotterAC(unsigned long ts, sh2_RotationVectorWAcc_t Mast, sh2_RotationVectorWAcc_t Boat) {
    teleplotter(ts, Mast.real, Mast.i, Mast.j, Mast.k, Boat.real, Boat.i, Boat.j, Boat.k);
    //Serial.printf(">Macc:%0.4ld:%0.2f\n", ts, Maccuracy*RADTODEG);
    //Serial.printf(">Mcal:%0.4ld:%d\n", ts, sensorValueMstatus);
    //Serial.printf(">Bacc:%0.4ld:%0.2f\n", ts, Baccuracy*RADTODEG);
    //Serial.printf(">Bcal:%0.4ld:%d\n", ts, sensorValue.status); // maybe in both?
}

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
        unsigned long ts = millis();
        switch (sensorValueMast.sensorId) {
        case SH2_ROTATION_VECTOR:
             mastCompassDeg = calculateHeading(sensorValueMast.un.geoMagRotationVector.real, sensorValueMast.un.geoMagRotationVector.i, sensorValueMast.un.geoMagRotationVector.j, sensorValueMast.un.geoMagRotationVector.k, mastOrientation);
            readCompassDelta();
            if (teleplot) {
                teleplotterAC(ts, sensorValueMast.un.geoMagRotationVector,sensorValue.un.geoMagRotationVector);
            }
            break;       
        case SH2_GAME_ROTATION_VECTOR:
            mastCompassDeg = calculateHeading(sensorValueMast.un.gameRotationVector.real, sensorValueMast.un.gameRotationVector.i, sensorValueMast.un.gameRotationVector.j, sensorValueMast.un.gameRotationVector.k, mastOrientation);
            readCompassDelta();
            if (teleplot) {
                teleplotterNAC(ts, sensorValueMast.un.gameRotationVector,sensorValue.un.gameRotationVector);
            }
            break;
        case SH2_ARVR_STABILIZED_GRV:
            mastCompassDeg = calculateHeading(sensorValueMast.un.arvrStabilizedGRV.real, sensorValueMast.un.arvrStabilizedGRV.i, sensorValueMast.un.arvrStabilizedGRV.j, sensorValueMast.un.arvrStabilizedGRV.k, mastOrientation);
            readCompassDelta();
            if (teleplot) {
                teleplotterNAC(ts, sensorValueMast.un.arvrStabilizedGRV,sensorValue.un.arvrStabilizedGRV);
            }
            break;
        default:
            Serial.printf("ESPNOW ondatarecv(): got unknown sensorID 0x%0x\n", sensorValueMast.sensorId);
            break;
        }
    } else { // first time hearing from peer
        Serial.printf("found peer\n");
        foundPeer = true;
        Serial.printf("channel: %d ifidx: %x encrypt: %d addr size %d (6)\n", peerInfo.channel, peerInfo.ifidx, peerInfo.encrypt, sizeof(peerInfo.peer_addr));
        Serial.printf("ESP peer MAC addr: ");
        for (int i=0; i<ESP_NOW_ETH_ALEN; i++)
            Serial.printf("%02X ", mac_addr[i]);
        memcpy(compassAddress, peerInfo.peer_addr, ESP_NOW_ETH_ALEN);  // set compass address
        if (int err=esp_now_add_peer(&peerInfo) != ESP_OK) {
           logToAll("Failed to add compass peer: 0x" + String(err,HEX));
        } else logToAll("ESP-NOW compass peer added");
    }
}

// Send control message via ESP-NOW
// called from webserver.cpp when settings are updated "get /params"
bool sendMastControl() {
//  outCommand.compassOnToggle = compassOnToggle;
  outCommand.id = reportType;   // specify which bno085 report to use
//  outCommand.orientation = mastOrientation;
//  outCommand.frequency = mastFrequency;
  esp_err_t result = esp_now_send(compassAddress, (uint8_t *) &outCommand, sizeof(outCommand));
  if (result == ESP_OK) {
    logToAll("sent mast control report type 0x" + String(reportType,HEX));
    return true;
  } else {
    logToAll("Error sending the data: 0x" + String(result,HEX));
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
        logToAll("Failed to add broadcast peer:" + String(err) + " (OK if already added)");
    } else logToAll("ESP-NOW broadcast peer added");
    esp_err_t result;
    for (int retries = 0; retries < 5; retries++) {
        result = esp_now_send(broadcastAddress, (uint8_t *) &outCommand, sizeof(outCommand)); // contents don't matter until later
        if (result == ESP_OK) {
            logToAll("sent broadcast");
        } else {
            logToAll("Error sending broadcast: " + String(result));
        }
        delay(100);
    }
}
#endif