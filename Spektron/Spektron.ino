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
float ink1= 0.4218749999999947265625; //1500/3555.5555556;// W/bit (3555.56bit= 3.125V/3.6V*4096bit; //inkrement spektron, 0 - 3.125V)
float Voltage;
int n= 100;
float ADC_raw;

void setup() {
Serial.begin(115200);
}

void loop() {
analogReference(AR_INTERNAL);
analogReadResolution(12); // Auflösung ist default auf 10bit, hier wird auf 12bit gewechselt
/*
ADC_raw= analogRead(analogPin);
Serial.print("ADC_raw= ");
Serial.println(ADC_raw);
P= ADC_raw * ink1;
Serial.print("P= ");
Serial.println(P);
*/

for (int i = 0; i < n; i++)
  {
    ADC_raw += analogRead(analogPin);   //read peak voltage
  }
ADC_raw= ADC_raw/n;
P = ADC_raw * ink1;

/*Serial.print("\nVREF= ");
Serial.print(AR_INTERNAL);
Serial.println("");*/
Serial.print("\nADC_raw= ");
Serial.print(ADC_raw);
Serial.print("\nP_Einstrahlung= ");
Serial.print(P);/*
Serial.print("\nADC= ");
Serial.print(analogRead(A0));
Serial.print("\nEinstrahlung= ");
Serial.print(analogRead(A0) * ink1);
Serial.print("\ninkrement= ");
Serial.print(ink1);*/
Serial.println("");
delay(1000);
}
