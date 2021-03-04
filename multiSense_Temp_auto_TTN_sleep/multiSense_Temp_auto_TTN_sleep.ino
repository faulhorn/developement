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
#include <Adafruit_SleepyDog.h>

#define VBATPIN A6
byte bus;
byte sensor;
int sleepTime= 1000*60*5; //set sleeptime to 5 minutes
int sleepTimeMin= sleepTime/60000; //convert value from [ms] to [min]

//todo!
const byte one_wire_busses= 1; //number of busses, meaning number of channels on which the board will be able to hear DS18B20 sensors

//!! Cant use Pins 5, 6, 10, 11 since those are used by FeatherWing RFM95W LoRa module !!
const byte one_wire_pins[one_wire_busses]= {11}; //numbers of the pins that have DS18B20 sensors connected to them. this is an array, to ensure, multible busses with multible sensors are possible

const byte numOfTempSensors= 4; //total number of ds18b20 sensors on all busses

static const PROGMEM u1_t NWKSKEY[16] = { 0x5C, 0x78, 0xE5, 0x4E, 0xF3, 0xA8, 0x9D, 0x98, 0x0D, 0x90, 0xAC, 0xED, 0xF2, 0x76, 0x69, 0x6C };

static const u1_t PROGMEM APPSKEY[16] = { 0xFE, 0xC3, 0x4D, 0xBF, 0x3D, 0x0D, 0x1D, 0xEE, 0xD2, 0xB5, 0xE8, 0x3F, 0xDD, 0x8F, 0xB0, 0xE0 };

static const u4_t DEVADDR = 0x2601107E; // TODO, msb

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }
static uint8_t mydata[numOfTempSensors]; //payload mydata, array with 1 byte reserved per ds18b20 (uint8 can store 8 bits = 1 byte). measured value therefore cant be larger than maxvalue of 256
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

void sleep() {
  digitalWrite(LED_BUILTIN, LOW); // Show we're asleep
  int sleepMS = Watchdog.sleep(sleepTime);  // Sleep for up to 1 minute.
  // Code resumes here on wake.
  digitalWrite(LED_BUILTIN, HIGH); // Show we're awake again
  // Try to reattach USB connection on "native USB" boards (connection is
  // lost on sleep). Host will also need to reattach to the Serial monitor.
  // Seems not entirely reliable, hence the LED indicator fallback.
  #if defined(USBCON) && !defined(USE_TINYUSB)
    USBDevice.attach();
  #endif
  Serial.print("Slept for ");
  Serial.print(sleepTimeMin, DEC);
  Serial.println(" minutes.");
  Serial.println();
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void updateMeasurement() {
  //idea: measure 3 temperatures. 2 water temps inside the water tank on different height levels, 1 air temp outside the tank 
  Serial.print("mydata: ");
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
      Serial.print(": ");
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

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        updateMeasurement();
        LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Show we're awake
     
  Serial.begin(115200);
  while(!Serial); // wait for Arduino Serial Monitor (native USB boards)
  
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
  do_send(&sendjob);
  sleep();
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

  os_runloop_once();
}
