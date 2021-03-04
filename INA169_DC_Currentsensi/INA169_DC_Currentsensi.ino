/*
 * Messwerte schwanken stark!! Mit Multimetter messe ich bei EInstellung Speisegerät 5V / 2A genau 2A
 * Mit Sensor schwanken Werte zwischen 0.9 und 2.9 A!! Wegen Berechung? Falsch angeschlossen?
 *
 */
const int analogPin= A0;
const int RS= 0.1; //Shunt resistor value in ohms
const int Rload= 1000; //Load resistor value in ohms (for current source)
const int V_Ref= 3.3; // Voltage reference for analog read
float a; // for adc value
float current; // for current value
int n= 100;

void setup() {
Serial.begin(115200);
}

void loop() {
//analogReference(AR_INTERNAL_3_0); //Change Analog Reference Voltage -> AR_INTERNAL_3_0 (0.6V Ref * 5 = 0..3.0V)
//analogReadResolution(10);

/*
for (int i=0; i < n; i++) {
  a += analogRead(analogPin);
}
a= a/n;
*/

a= analogRead(analogPin);

Serial.println("ADC 10-bit : ");
Serial.println(a);

/*
a= (a / 1023) * V_Ref; // V_Ref equals 1023 bits, ADC Bit output equals how much?

// Follow the equation given by the INA169 datasheet to
// determine the current flowing through RS. Assume RL = 1k
// Is = (Vout x 1k) / (RS x RL)
current = a / (Rload * RS);

Serial.print("Current= ");
Serial.print(a, 3);
Serial.println("A");
*/
delay(1000);
}
