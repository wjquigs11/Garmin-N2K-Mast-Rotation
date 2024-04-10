/*
Parse magnetic heading (typically will come from pypilot)
compare to magnetic heading sent via BLE from ESP32 with compass on mast
transmit on n2k as rudder angle for rudder #2
*/
#include <Arduino.h>
#include <N2kMessages.h>
#include <NMEA2000_esp32.h>
#include <ReactESP.h>
#include <Wire.h>
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>
#include <vector>
#include <numeric>
#include <movingAvg.h>
#include <SPI.h>
#include "windparse.h"
#include <Arduino_JSON.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

//int mastRotate, rotateout;
extern String hostname;
BLEServer *pServer;
BLEService *pService;
BLECharacteristic *pMagOrientation;

//String bleServerName(128);
#define bleServerName "BLEcompass"

/* UUID's of the service, characteristic that we want to read*/
// BLE Service
static BLEUUID bmeServiceUUID("01a91d81-b2be-4473-8490-5f6089a92b54");
static BLEUUID magneticOrientationUUID("de64c403-2b3e-4566-bcea-e291922b934d");

//Flags stating if should begin connecting and if the connection is up
static boolean doConnect = false;
static boolean connected = false;

//Address of the peripheral device. Address will be found during scanning...
static BLEAddress *pServerAddress;
 
//Characteristicd that we want to read
static BLERemoteCharacteristic* magOrientCharacteristic;

//Activate notify
const uint8_t notificationOn[] = {0x1, 0x0};
const uint8_t notificationOff[] = {0x0, 0x0};

char* magOrientationChar; // maybe we don't need this go direct to int
int magOrientation;

//Flags to check whether new readings are available
boolean newMagOrient = false;

bool connectToServer(BLEAddress pAddress);
static void magNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);

void MagHeading(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  double heading;
  double deviation;
  double variation;
  tN2kHeadingReference headingRef;
  tN2kMsg correctN2kMsg; 

  if (ParseN2kPGN127250(N2kMsg, SID, heading, deviation, variation, headingRef) ) {
    if (headingRef == N2khr_magnetic) {
        // compare to heading reading from mast compass (via BLE)
        Serial.print("compass heading: ");
        Serial.print(heading);
        Serial.print(" mast orientation: ");
        Serial.println(magOrientation);
    } else
      Serial.println("not magnetic heading from main compass; doing nothing");
  }
}

// Callback function that gets called, when another device's advertisement has been received
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.getName() == bleServerName) { // Check if the name of the advertiser matches
      advertisedDevice.getScan()->stop(); // Scan can be stopped, we found what we are looking for
      pServerAddress = new BLEAddress(advertisedDevice.getAddress()); // Address of advertiser is the one we need
      doConnect = true; // Set indicator, stating that we are ready to connect
      // do I need this if we connect here? Why did they connect in loop?
      Serial.println("Device found. Connecting!");
      if (connectToServer(*pServerAddress)) {
        Serial.println("We are now connected to the BLE Server.");
        // Activate the Notify property of each Characteristic
        magOrientCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
        connected = true;
      } else {
        Serial.println("We have failed to connect to the server; Restart your device to scan for nearby BLE server again.");
      }
    }
  }
};
 
void SetupBLE() {
  //Init BLE device
  BLEDevice::init("");
 
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 30 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(30);
}

//Connect to the BLE Server that has the name, Service, and Characteristics
bool connectToServer(BLEAddress pAddress) {
   BLEClient* pClient = BLEDevice::createClient();
 
  // Connect to the remove BLE Server.
  pClient->connect(pAddress);
  Serial.println(" - Connected to server");
 
  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(bmeServiceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(bmeServiceUUID.toString().c_str());
    return (false);
  }
  // Obtain a reference to the characteristics in the service of the remote BLE server.
  magOrientCharacteristic = pRemoteService->getCharacteristic(magneticOrientationUUID);

  if (magOrientCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID");
    return false;
  }
  Serial.println(" - Found our characteristic");
 
  //Assign callback functions for the Characteristics
  magOrientCharacteristic->registerForNotify(magNotifyCallback);
  return true;
}

//When the BLE Server sends a new magnetic reading with the notify property
static void magNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  magOrientationChar = (char*)pData;
  magOrientation = atoi(magOrientationChar);
  newMagOrient = true;
}
/*
void magLoop() {
  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are
  // connected we set the connected flag to be true.
  //if (doConnect == true) {
  // if new readings are available, compare to last mag heading from n2k
  // since this is called right after n2k->parsemessages we should have a new reading
  if (newMagOrient){
    newMagOrient = false;
    printReadings();
  }
}
*/
