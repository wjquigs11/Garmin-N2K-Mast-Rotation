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
// ESP32
// purple wire SDA 21
// orange wire SCL 22
// SH-ESP32 16/17
// ESPberry
// SDA RPI GPIO2 IO21
// SCL RPI GPIO3 IO22

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

uint8_t compassAddress[] = {0x08, 0xB6, 0x1F, 0xB8, 0xE1, 0xD4};
esp_now_peer_info_t peerInfo;

// struct we will receive from compass
// not using now because we're getting mast heading as PGN on wind bus
typedef struct compass_s {
  int id;
  float heading;
  float accuracy;
  int calStatus;
  int readingId;
} compass_s;
compass_s inReadings;

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
/*
#ifdef DEBUG
  // Copies the sender mac address to a string
  char macStr[18];
  Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
#endif
  memcpy(&inReadings, incomingData, sizeof(inReadings));
  
  board["id"] = inReadings.id;
  board["heading"] = inReadings.heading;
  board["accuracy"] = inReadings.accuracy;
  board["calStatus"] = inReadings.calStatus;
  board["readingId"] = String(inReadings.readingId);
  String jsonString = JSON.stringify(board);
  events.send(jsonString.c_str(), "new_readings", millis());
//#define DEBUG
#ifdef DEBUG  
  Serial.printf("Board ID %u: %u bytes\n", inReadings.id, len);
  Serial.printf("heading: %.2f\n", inReadings.heading);
  Serial.printf("accuracy: %.2f\n", inReadings.accuracy);
  Serial.printf("calStatus: %d\n", inReadings.calStatus);
  Serial.printf("readingID value: %d \n", inReadings.readingId);
  Serial.println();
#endif
  // set mastCompass to the heading value from the mast compass
  mastCompassDeg = inReadings.heading;
  readings["mastHeading"] = mastCompassDeg;
  // set boatHeading to local compass reading (currently CMPS14)
  // windparse will calculate corrected AWA based on boat compass+mast compass
  boatCompassDeg = getCompass(mastOrientation);
  readings["boatHeading"] = boatCompassDeg;
  Serial.printf("ESPNOW Mast Heading: %.2f Boat Heading: %.2f\n", mastCompassDeg, boatCompassDeg);
*/
}

// Send control message via ESP-NOW
void sendMastControl() {
  outCommand.compassOnToggle = compassOnToggle;
  outCommand.orientation = mastOrientation;
  outCommand.frequency = mastFrequency;
  esp_err_t result = esp_now_send(compassAddress, (uint8_t *) &outCommand, sizeof(outCommand));
  if (result == ESP_OK) {
    Serial.printf("sent mast control\n");
  } else {
    Serial.printf("Error sending the data: %d\n", result);
  }
}

// Callback when data is sent
String success;
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
/*
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>ESP-NOW DASHBOARD</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="stylesheet" href="https://use.fontawesome.com/releases/v5.7.2/css/all.css" integrity="sha384-fnmOCqbTlWIlj8LyTjo7mOUStjsKC4pOpQbqyi7RrhN7udi9RwhKkMHpvLbHG9Sr" crossorigin="anonymous">
  <link rel="icon" href="data:,">
  <style>
    html {font-family: Arial; display: inline-block; text-align: center;}
    p {  font-size: 1.2rem;}
    body {  margin: 0;}
    .topnav { overflow: hidden; background-color: #2f4468; color: white; font-size: 1.7rem; }
    .content { padding: 20px; }
    .card { background-color: white; box-shadow: 2px 2px 12px 1px rgba(140,140,140,.5); }
    .cards { max-width: 700px; margin: 0 auto; display: grid; grid-gap: 2rem; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); }
    .reading { font-size: 2.8rem; }
    .packet { color: #bebebe; }
    .card.temperature { color: #fd7e14; }
    .card.humidity { color: #1b78e2; }
  </style>
</head>
<body>
  <div class="topnav">
    <h3>ESP-NOW DASHBOARD</h3>
  </div>
  <div class="content">
    <div class="cards">
      <div class="card heading">
        <h4><i class="fas fa-thermometer-half"></i>HEADING</h4><p><span class="reading"><span id="t1"></span> &deg;</span></p><p class="packet">Reading ID: <span id="rt1"></span></p>
      </div>
      <div class="card accuracy">
        <h4><i class="fas fa-tint"></i>ACCURACY</h4><p><span class="reading"><span id="h1"></span>RADs</span></p><p class="packet">Reading ID: <span id="rh1"></span></p>
      </div>
      <div class="card cal_status">
        <h4><i class="fas fa-thermometer-half"></i>CALIBRATION</h4><p><span class="reading"><span id="t2"></span></span></p><p class="packet">Reading ID: <span id="rt2"></span></p>
      </div>
    </div>
  </div>
<script>
if (!!window.EventSource) {
 var source = new EventSource('/events');
 
 source.addEventListener('open', function(e) {
  console.log("Events Connected");
 }, false);
 source.addEventListener('error', function(e) {
  if (e.target.readyState != EventSource.OPEN) {
    console.log("Events Disconnected");
  }
 }, false);
 
 source.addEventListener('message', function(e) {
  console.log("message", e.data);
 }, false);
 
 source.addEventListener('new_readings', function(e) {
  console.log("new_readings", e.data);
  var obj = JSON.parse(e.data);
  document.getElementById("t"+obj.id).innerHTML = obj.temperature.toFixed(2);
  document.getElementById("h"+obj.id).innerHTML = obj.humidity.toFixed(2);
  document.getElementById("rt"+obj.id).innerHTML = obj.readingId;
  document.getElementById("rh"+obj.id).innerHTML = obj.readingId;
 }, false);
}
</script>
</body>
</html>)rawliteral";
*/

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
  }
/* turning off for now to save space because I'm not using ESPnow for compass data
  server.on("/espnow", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });
*/
}

// get heading from (local) compass
// called when we get a Heading PGN on the wind bus (which means mast compass transmitted)
// so frequency of update is going to depend on how often we get a Heading PGN from the mast
// also called as a Reaction in case we're not connected to mast compass
float getCompass(int correction) {
  //Serial.println("getCompass");
  if (cmps14_ready) {}
  Wire.beginTransmission(CMPS14_ADDRESS);  // starts communication with CMPS14
  Wire.write(ANGLE_8);                     // Sends the register we wish to start reading from
  Wire.endTransmission();
  //if (!error) { // we found CMPS14
    // Request 5 bytes from the CMPS12
    // this will give us the 8 bit bearing, 
    // both bytes of the 16 bit bearing, pitch and roll
    Wire.requestFrom(CMPS14_ADDRESS, 5); 
    while((Wire.available() < 5)); // (hang)
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
    } else */
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
