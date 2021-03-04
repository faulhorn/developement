//const float bitsProA= 184.61538461538461538461538461538;
const float bitsProA= 177.195;
//const float bitsProA= 204.75;
float peakVoltage;
float ADC;
const int n= 1000;

void setup() {
  // open a serial connection
  Serial.begin(9600);

}

void loop() {
  // read the input on A0 at default resolution (10 bits)
  // and send it out the serial connection

  analogReference(AR_INTERNAL_3_0); //Change Analog Reference Voltage -> AR_INTERNAL_3_0 (0.6V Ref * 5 = 0..3.0V)

  analogReadResolution(10);
  Serial.println("ADC 10-bit (default) : ");
  Serial.println(analogRead(A0));
  

  // change the resolution to 12 bits and read A0
  analogReadResolution(12);
  Serial.println("ADC 12-bit : ");
     for (int i = 0; i < n; i++)
  {
    ADC += analogRead(A0)-100;   //read peak voltage
  }

  ADC= ADC/n;
  Serial.println(ADC);

   for (int i = 0; i < n; i++)
  {
    peakVoltage += analogRead(A0)-100;   //read peak voltage
  }

  peakVoltage= peakVoltage/n;

  Serial.println("Strom= ");
  Serial.println(peakVoltage/bitsProA);
  

/* 
// change the resolution to 16 bits and read A0
  analogReadResolution(16);
  Serial.print(", 16-bit : ");
  Serial.print(analogRead(A0));

  // change the resolution to 8 bits and read A0
  analogReadResolution(8);
  Serial.print(", 8-bit : ");
  Serial.println(analogRead(A0));

  // a little delay to not hog Serial Monitor
  */
  delay(1000);
}
