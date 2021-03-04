/*
 * Autor: Philipp Husi
 * Zweck: Auslesung Einstrahlungssensor Spektron 300
 * Spannungsausgang= 3125mV= 3.125V
 * Einstrahlung zwischen 0 - 1500W
 * Auf nrf: 0 - 3.6V -> 4096 inkremente. Von Spektron kommt 0 - 3.125V -> 3555.5555556 inkremente
 */

//#define VREF
const int analogPin= A1;
float P; //Leistung Einstrahlung in W/m^2
float offset= 800; //bits, -> =4mA, gemessen ohne einstrahlung
float ink1= 0.45565006075334143377886; //1500/3292;// W/bit (3292bit= 4092 (max) - 800(offset); (inkrement spektron, 4 - 20mA)
int n= 100000;
float ADC_raw;
float ADC_raw_offset;

void setup() {
Serial.begin(115200);
}

void loop() {
analogReference(AR_INTERNAL); //Vref= 3.6V
analogReadResolution(12); // Auflösung ist default auf 10bit, hier wird auf 12bit gewechselt also von Bereich 1024bits zu 4096bits
for (int i = 0; i < n; i++)
  {
    ADC_raw += analogRead(analogPin);   //read peak voltage
  }

ADC_raw= ADC_raw/n - offset;
if (ADC_raw < 0) {
  ADC_raw= 0;
  P= 0;
}
else {
P = ADC_raw * ink1;
}
Serial.print("\nADC_raw= ");
Serial.print(ADC_raw);
Serial.print("\nP_Einstrahlung= ");
Serial.print(P);
Serial.println("");
//delay(1000);
}
