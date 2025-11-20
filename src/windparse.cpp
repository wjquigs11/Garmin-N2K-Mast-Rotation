#include "include.h"
#include "windparse.h"

#define CAN_TX_PIN GPIO_NUM_33  //The library defines as default Tx pin to GPIO 16 and Rx pint to GPIO 4. You can change these with defines:
#define CAN_RX_PIN GPIO_NUM_32
//#define USE_N2K_CAN 7  // for use with ESP32
tNMEA2000 *n2kWind;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable n2kWind object
#include <NMEA2000_esp32.h>
#include <N2kMessages.h>

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);       // Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

// define the moving average object
// 10 samples for now. More samples means less noise but it also means delay in sensor catching up to mast position
// when you change sampling to 10 or 100 msecs, adjust accordingly
movingAvg honeywellSensor(100);                

using namespace std;

// This is required on ESP32 to put the ISR in IRAM. Define as
// empty for other platforms. Be careful - other platforms may have
// other requirements.
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

volatile bool new_data = false;
void IRAM_ATTR NewDataReadyISR() {
  new_data = true;
}

tNMEA2000Handler n2kWindHandlers[]={
  {130306L,&WindSpeed},
  {0,0}
};

// Initialize static variables for RotationSensor Class
int RotationSensor::newValue{0};
int RotationSensor::oldValue{0};

double WindSensor::windSpeedKnots{0.0};
int WindSensor::windAngleDegrees{0};

// List here messages your device will transmit.
const unsigned long TransmitMessages[] PROGMEM={130306L,0};   // This is the PGN for Wind

// the corrected AWA after applying mast rotation
int rotateout;

void setupWind() {
  log::toAll("setting up N2K wind bus");
  n2kWind = new tNMEA2000_esp32(CAN_TX_PIN, CAN_RX_PIN);
  // Set Product information
  n2kWind->SetProductInformation("00000002",                // Manufacturer's Model serial code
                                 100,                       // Manufacturer's product code
                                 "Mast Rotation Compensator",     // Manufacturer's Model ID
                                 "2.0B (2022-07-17)",  // Manufacturer's Software version code
                                 "1.1.0.0 (2022-07-17)"     // Manufacturer's Model version
                                 );
  // Set device information
  n2kWind->SetDeviceInformation(1, // Unique number. Use e.g. Serial number.
                                130, // Device function=Atmospheric. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                85, // Device class=External Environment. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2046 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf                               
                               );

  
  //n2kWind->SetMode(tNMEA2000::N2km_NodeOnly,23);    // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly
  n2kWind->SetMode(tNMEA2000::N2km_ListenAndNode,23);    // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly
  n2kWind->EnableForward(false);
  n2kWind->SetMsgHandler(HandleNMEA2000Msg);
  n2kWind->ExtendTransmitMessages(TransmitMessages);
  n2kWind->Open();

  honeywellSensor.begin();    //Instantiates the moving average object

  ads.begin();
}

void loopWind() {
    // No need to parse the messages at every single loop iteration; 1 ms will do 
    //PollCANStatus();
    n2kWind->ParseMessages();
    //actisense_reader.ParseMessages();
    int windInput = readWindAngleInput();                     //from parsed NMEA string
    int mastRotate = readAnalogRotationValue();
    int anglesum = windInput + mastRotate;                    //adds windinput and mastrotate

    if (anglesum<0) {                             //ensure sum is 0-359
        rotateout = anglesum + 360;
    }              
    else if (anglesum>359) {   
        rotateout = anglesum - 360;               
    }
    else {
        rotateout = anglesum;               
    }
    //SendN2kWind(rotateout);
}
        //OLEDdataWind(mastRotate);

double ReadWindAngle(int rotateout) {
  return DegToRad(rotateout); // 
}

double ReadWindSpeed() {
  return WindSensor::windSpeedKnots;  //Read the parsed windspeed from NMEA stream
}

int readWindAngleInput() {
  return WindSensor::windAngleDegrees;
}

static int lowPot=65535;
static int highPot=-1;
static int lowset = 272;
static int highset = 2350;
/* note: the Phiher sensor appears to jump when it goes out of range;
    after it reaches 'highset' the value drops and after it reaches 'loweset' the value goes up.
    I can probably use this to fix the rotation correction at +/-50
*/
static int counter=0;

int readAnalogRotationValue() {      //returns mastRotate value when called. This is in degrees, and corresponds to the current value of the Honeywell sensor
  int adc1 = (ads.readADC_SingleEnded(0)>>1);   // chop off a bit to reduce noise
  if (adc1 < lowPot) lowPot = adc1;
  if (adc1 > highPot) highPot = adc1;
  int newValue = honeywellSensor.reading(adc1);    // calculate the moving average
  int oldValue = RotationSensor::oldValue;
  
  if (newValue < highset){                 //writes value to oldsensor if below highset threshold
    oldValue = newValue;
  }

  // Update values for new and old values (for the next loop iteration)
  RotationSensor::newValue = newValue;
  RotationSensor::oldValue = oldValue;

  // not sure why he's only mapping the old value if newvalue<highset
  //int mastRotate = map(oldValue, lowset, highset, -50, 50);    
  int mastRotate = map(newValue, lowset, highset, -50, 50);    
  if (counter++ % 100 == 0)
    Serial.printf("low %d high %d reading %d average %d rotateout %d\n", lowPot, highPot, adc1, newValue, mastRotate);
  return mastRotate;
}

void WindSpeed(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    double windSpeedMetersSeconds;
    double windAngle;
    tN2kWindReference WindReference;

    if (ParseN2kWindSpeed(N2kMsg,SID, windSpeedMetersSeconds, windAngle, WindReference) ) {
      double windSpeedKnots = windSpeedMetersSeconds * 1.94384; //maybe * .01?
      int windAngleDegrees = windAngle * 57.2958; //maybe * .0001? 

      // Update Static Object Values for Wind Velocity and Angle
      WindSensor::windSpeedKnots = windSpeedKnots;
      WindSensor::windAngleDegrees = windAngleDegrees;
      
     }
}

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {   //NMEA 2000 message handler
  int iHandler;
  // Find handler
  log::toAll(String(N2kMsg.PGN));
  for (iHandler=0; n2kWindHandlers[iHandler].PGN!=0 && !(N2kMsg.PGN==n2kWindHandlers[iHandler].PGN); iHandler++);
  if (n2kWindHandlers[iHandler].PGN!=0) {
    n2kWindHandlers[iHandler].Handler(N2kMsg); 
  }
}

void OLEDdataWind(int mastrotate) {                           // Data OLED Screen
  display.clearDisplay();
  display.setTextSize(7);
  display.setTextColor(WHITE);
  display.setCursor(0, 0); 
  display.println(mastrotate);
  display.display();
}

void OLEDdataWindDebug(int mastrotate, int rotateout) {                           // Data OLED Screen
  double windSpeedKnots = WindSensor::windSpeedKnots;
  int windAngleDegrees = WindSensor::windAngleDegrees;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println(windSpeedKnots);
  display.setCursor(30, 0);
  display.println("WindSpeed");
  display.setCursor(0, 12);
  display.println(windAngleDegrees);
  display.setCursor(30, 12);
  display.println("WindAngle");
  display.setCursor(0, 24);
  display.println(rotateout);
  display.setCursor(30, 24);
  display.println("Adj WindAngle");
  display.setCursor(0, 36);
  display.println(mastrotate);
  display.setCursor(30, 36);
  display.println("Mast Rotation");
  display.display();
}

void OLEDdataSplash() {                    // Splash OLED Screen
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Mast Rotation");
  display.setCursor(0, 12);
  display.println("Compensator");
  display.setCursor(0, 24);
  display.println("2.0B");
  display.display();
  delay(1000);                // waits 
}

void SendN2kWind(int rotateout) {
  static unsigned long WindUpdated=millis();
  tN2kMsg N2kMsg;

  if ( WindUpdated+WindUpdatePeriod<millis() ) {
    SetN2kWindSpeed(N2kMsg, 1, ReadWindSpeed(), ReadWindAngle(rotateout),N2kWind_Apprent);  // Typo in N2kWindApprent is intentional
    WindUpdated=millis();
    n2kWind->SendMsg(N2kMsg);
  }
}
