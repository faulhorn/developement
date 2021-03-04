/*****************************************************************************************************************
 *
 *Testing: mydata is initializd and sent but without content
 *Next step: correctly writing measurements into mydata
 *
 *About DallasTemperature: https://www.arduino.cc/reference/en/libraries/dallastemperature/
 *About OneWire: https://www.arduino.cc/reference/en/libraries/onewire/
 *About arrays: https://www.arduino.cc/reference/en/language/variables/data-types/array/
 *About datatype byte: A byte stores an 8-bit unsigned number, from 0 to 255. so "byte" is equal (in size) to uint8 or uint8[1].
*****************************************************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <OneWire.h> //controls the datapin and does the heavy lifting
#include <DallasTemperature.h> //knows all the commands, the ds18b20 knows, "uses" OneWire.h to drive teh ds18b20
#include <SoftwareSerial.h> 

#define VBATPIN A6
byte bus;
byte sensor;

//todo!
const byte one_wire_busses= 1; //number of busses, meaning number of channels on which the board will be able to hear DS18B20 sensors

//!! Cant use Pins 5, 6, 10 since those are used by FeatherWing RFM95W LoRa module !!
const byte one_wire_pins[one_wire_busses]= {11}; //number of the pins that have DS18B20 sensors connected to them. this is an array, to ensure, multible busses with multible sensors are possible

const byte numOfTempSensors= 4; //total number of ds18b20 sensors on all busses
const byte numOfDistSensors= 1; //total number of A02YYUW sensors
const byte n= numOfTempSensors + numOfDistSensors;

static const PROGMEM u1_t NWKSKEY[16] = { 0x72, 0x83, 0x09, 0x62, 0x2D, 0x71, 0x49, 0x28, 0x09, 0xAA, 0x6E, 0xE5, 0x81, 0x4F, 0x64, 0x3D };

static const u1_t PROGMEM APPSKEY[16] = { 0xB3, 0x28, 0x58, 0x45, 0x8A, 0x7A, 0xEA, 0x8A, 0xDC, 0x6F, 0x54, 0x6A, 0x1F, 0xF1, 0x4B, 0xB7 };

static const u4_t DEVADDR = 0x26011400; // TODO, msb

#define mess_TX 12 // TX mapped to digital pin 12 on nrf52840 feather express
#define mess_RX 13 // RX mapped to digital pin 13 on nrf52840 feather express

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }
unsigned char data[4]={}; 
float distance; 
static uint8_t mydata[n]; //payload mydata, array with 1 byte reserved per sensor
static osjob_t sendjob;
const unsigned TX_INTERVAL = 60; // sec
// Pin mapping, see https://github.com/tamberg/fhnw-iot/wiki/FeatherWing-RFM95W
// Feather nRF52840 with FeatherWing RFM95W
const lmic_pinmap lmic_pins = {
  .nss = 5, // E = CS
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 6, // D = RST
  .dio = {
    10, // B = DIO0 = IRQ 
    9, // C = DIO1
    LMIC_UNUSED_PIN
  },
};
OneWire busses[one_wire_busses]; //create OneWire Object (=Instance) busses (array) from OneWire library. number of elements equals number of one_wire_busses
DallasTemperature sensors[one_wire_busses]; //create sensors object (array) from DallasTemperature library. number of elements equals number of one_wire_busses
SoftwareSerial mySerial(mess_RX,mess_TX); // RX, TX 

/*
void onEvent (ev_t ev) {

}*/

void distanceSense() {
  do{ 
      for(int i=0;i<4;i++) { 
        data[i]=mySerial.read(); 
        } 
      }
      
  while(mySerial.read()==0xff); 
  mySerial.flush(); 
      if(data[0]==0xff) { 
        int sum; 
        sum=(data[0]+data[1]+data[2])&0x00FF; 
        
        if(sum==data[3]) { 
          distance=(data[1]<<8)+data[2]; 
          
          if(distance>300) { //Minimaldistanz auf 300mm gesetzt
          Serial.print("distance="); 
          //Serial.print(distance/10); 
          int distanceInt= round(distance); //decimal places unnecessary, distance is rounded and packet into integer
          mydata[numOfTempSensors]= distanceInt; //distance is written in mydata at next free position after the temperatures
          Serial.print(mydata[numOfTempSensors]);
          //Serial.print(" cm"); 
          }
          else { 
                Serial.println("Below the lower limit"); 
               } 
        }
        else 
        Serial.println("ERROR"); 
      } 
      Serial.println("");
      delay(150); 
}

void tempSense() {
  Serial.print("Temps: ");
  for (bus= 0; bus < one_wire_busses; bus++) {
    Serial.print("Bus ");
    Serial.print(bus+1); //for reading purposes displaying the count starting from 1 instad of 0
    Serial.print(" ");
    sensors[bus].requestTemperatures();  //command to get temperature readings
    delay(250);

    for (sensor= 0; sensor < sensors[bus].getDS18Count(); sensor++) { //replaced "getDeviceCount() with "getDS18Count()" to avoid unwanted count of other onewire sensors
      //getDeviceCount() returns the number of devices found on the bus
      Serial.print("Sensor ");
      Serial.print(sensor+1); //for reading purposes displaying the count starting from 1 instad of 0
      Serial.print(" ");
      mydata[sensor]= sensors[bus].getTempCByIndex(sensor);
      Serial.print(mydata[sensor]);
      //Serial.print(sensors[bus].getTempCByIndex(sensor));
      Serial.print("°C ");
    }
  }
  Serial.println("");
  //Serial.print("Succsessfully created payload mydata of length= ");
  //Serial.println(sensor);
}

void updateMeasurement() {
  tempSense();
  distanceSense();
  //Serial.print("distance="); Serial.print(mydata[numOfTempSensors]); Serial.println(" cm"); 
}

/*
void do_send(osjob_t* j){
  
}*/

void setup() {
  //byte bus;    
  Serial.begin(115200);
  mySerial.begin(9600); //nötig?
  
  for (bus= 0; bus < one_wire_busses; bus++) {
    busses[bus].begin(one_wire_pins[bus]);    //initialize onewire bus
    sensors[bus].setOneWire(&(busses[bus]));  //setup DallasTemp lib; set onewire bus to be used
    sensors[bus].begin();                     //initialize ds18b20 sensors
    sensors[bus].setResolution(9);             //set resolution to 9 bit, LSB = 0.5°C, conversion time 97.75ms per sensor according to datasheet
    sensors[bus].setWaitForConversion(false); //tell library not to wait until conversion is finished. waiting time is and must be controlled by user
  }
  Serial.println("Setup successful.");
  pinMode(13, OUTPUT);
  //while (!Serial); // wait for Serial to be initialized
  Serial.begin(115200);
  delay(100);     // per sample code on RF_95 test
  Serial.println(F("Starting"));

 
  
  /*
  os_init();
  LMIC_reset();

  #ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);
  #else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);
  #endif

  #if defined(CFG_eu868)
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  #else
  # error Region not supported
  #endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink
  LMIC_setDrTxpow(DR_SF7,14);

  // Start job
  do_send(&sendjob);*/
}


void loop() {
  unsigned long now;
  now = millis();
  if ((now & 512) != 0) {
    digitalWrite(13, HIGH);
  }
  else {
    digitalWrite(13, LOW);
  }

  updateMeasurement();
  os_runloop_once();
}
