// 2 Tempsensoren, 1 Distsensor
// Entwicklung:
// Mindestdistanz in if Statement anpassen 
 

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <OneWire.h>
#include <SoftwareSerial.h> 

#define mess_TX A0 // Trage deinen Anschluss für das grüne Kabel ein 
#define mess_RX A1 // Trage deinen Anschluss für das blaue Kabel ein

OneWire ds(5); //todo (a 4.7K resistor is necessary)
//OneWire ds_2(); //todo (a 4.7K resistor is necessary)
//OneWire ds_3(); //todo (a 4.7K resistor is necessary)
SoftwareSerial mySerial(mess_RX,mess_TX); // RX, TX 

unsigned char data[4]={}; 
float distance; 
static uint8_t mydata[6];

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
          Serial.print(distance/10); 
          Serial.println("cm");
          int distanceInt= round(distance); //decimal places unnecessary, distance is rounded and packet into integer
          mydata[0] = highByte(distanceInt);
          mydata[1] = lowByte(distanceInt);
          }
          else { 
                Serial.println("Below the lower limit"); 
               } 
        }
        else 
        Serial.println("ERROR"); 
      } 
      //delay(150); 
}

void tempSense() { //should work for multible DS18B20 sensors
  byte bus;
  byte sensor;

  for (bus= 0; bus < one_wire_busses; bus++) {
    Serial.print("Bus ");
    Serial.print(bus);
    Serial.print(" ");
    sensors[bus].requestTemperatures();  //command to get temperature readings
    delay(250);

    for (sensor= 0; sensor < sensors[bus].getDeviceCount(); sensor++) {
      Serial.print("Sensor ");
      Serial.print(sensor);
      Serial.print(" ");
      Serial.print(sensors[bus].getTempCByIndex(sensor));
      Serial.print("°C ");
      tempC= sensors[bus].getTempCByIndex(sensor));
      int tempCInt = 100.0 * tempC; //to eliminate decimal places. celsius must be divided by 100 again in TTN or Node-Red to get the actual value
      mydata[2] = highByte(celsiusInt);
      mydata[3] = lowByte(celsiusInt);
    }
  }
  Serial.println("");
   
}

/*void updateMeasurement() {
  tempSense();
  distanceSense();
}*/

/*void onEvent (ev_t ev) {
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
}*/

/*void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        updateMeasurement();
        LMIC_setTxData2(1, mydataTemp, sizeof(mydata), 0);
        Serial.println(F("Packet queued"));
        LMIC_setTxData2(1, mydataDist
        $, sizeof(mydata), 0);
        Serial.println(F("Packet queued"));
        }
    }
    // Next TX is scheduled after TX_COMPLETE event.
}*/

void setup() {
  byte bus;
  Serial.begin(115200); 
  mySerial.begin(9600); 
  for (bus= 0; bus < one_wire_busses; bus++) {
    busses[bus].begin(one_wire_pins[bus]);    //initialize onewire bus
    sensors[bus].setOneWire(&(busses[bus]));  //setup DallasTemp lib; set onewire bus to be used
    sensors[bus].begin();                     //initialize ds18b20 sensors
    sensors[bus].setResolution(9);             //set resolution to 9 bit, LSB = 0.5°C, conversion time 97.75ms per sensor according to datasheet
    sensors[bus].setWaitForConversion(false); //tell library not to wait until conversion is finished. waiting time is and must be controlled by user
  }
  //lora zeug noch rein
  Serial.println("Setup successful.");
}

void loop() {
  tempSense();
  distanceSense();
  delay(1000);
}
