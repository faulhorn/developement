/*!
 * @file readACCurrent.
 * @n This example reads Analog AC Current Sensor.

 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (https://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @get from https://www.dfrobot.com

 Created 2016-3-10
 By berinie Chen <bernie.chen@dfrobot.com>

 Revised 2019-8-6
 By Henry Zhao<henry.zhao@dfrobot.com>

 https://wiki.dfrobot.com/Gravity_Analog_AC_Current_Sensor__SKU_SEN0211_
*/

const int ACPin = A0;         //set arduino signal read pin
#define ACTectionRange 20;    //set Non-invasive AC Current Sensor tection range (5A,10A,20A)

// VREF: Analog reference
// For Arduino UNO, Leonardo and mega2560, etc. change VREF to 5
// For Arduino Zero, Due, MKR Family, ESP32, etc. 3V3 controllers, change VREF to 3.3
#define VREF 3.3

//************************************************************************
/*
//Husischer testbereich
uint16 readADCValue()
{
  float peakVoltage = 0;  
  float ADCValue;
  for (int i = 0; i < 5; i++)
  {
    peakVoltage += analogRead(ACPin);   //read peak voltage
    delay(1);
  }
  peakVoltage = ADCValue;

  return ADCValue;
}


//************************************************************************
*/
float readACCurrentValue()
{
  float ACCurrtntValue = 0;
  float peakVoltage = 0;  
  float voltageVirtualValue = 0;  //Vrms
  float ADCValue;
  for (int i = 0; i < 5; i++)
  {
    peakVoltage += analogRead(ACPin);   //read peak voltage
    delay(1);
  }
  peakVoltage = ADCValue;
  peakVoltage = peakVoltage / 5; //hier wird gemittelt! oben wird 5 mal ausgelesen und aufaddiert. hier wird dann gemittelt 
  voltageVirtualValue = peakVoltage * 0.707;    //change the peak voltage to the Virtual Value of voltage, faktor 0.7 wegen effektivwert!

  /*The circuit is amplified by 2 times, so it is divided by 2.*/
  voltageVirtualValue = (voltageVirtualValue / 1024 * VREF ) / 2;  //VREF_default= 3.3V

  ACCurrtntValue = voltageVirtualValue * ACTectionRange;

  return ACCurrtntValue;
}

void setup() 
{
  Serial.begin(115200);
  pinMode(13, OUTPUT);

}

void loop() 
{
  analogReadResolution(12); //analogRead() output auf 12 bit umstellen (default ist 10bit aber nrf52840 feather express hat 12 bit adc)

  float ADC= analogRead(ACPin);
  /*
  float ADCValue = readADCValue();
  Serial.print("voltageADC= ");
  Serial.print(ADCValue);
  */
  Serial.print("AC_Pin= ");
  Serial.println(ADC);
  Serial.println("\n");
  
  Serial.print("VREF= ");
  Serial.println(VREF);
  Serial.println("\n");
  
  float ACCurrentValue = readACCurrentValue(); //read AC Current Value
  Serial.print(ACCurrentValue);
  Serial.println(" A");
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  delay(500);
}
