
const float res= 0.01996007984031936127744510978044; // resolution = 10A / 501bits = 0.01996A/bit
const float offset= 1880; // @ 0A 463 bits are measured by adc
const int a= A0; //analog Pin
float I_Amp= 0;
float ADC_raw;
int n=10000;

void setup() {
Serial.begin(115200);
analogReadResolution(12); //setting = 14bit ADC resolution
analogReference(AR_INTERNAL); // Vref= 3.6V
//analogReference(AR_INTERNAL_3_0);
}

void loop() {
//analogReadResolution(10);
//analogReference(AR_INTERNAL_3_0);

for (int i=0; i<n; i++) {
  ADC_raw+= analogRead(a);
}
ADC_raw= ADC_raw/n;

//ADC_raw= analogRead(a);
Serial.print("ADC_raw= ");
Serial.println(ADC_raw);
I_Amp= (ADC_raw - offset) * res;
Serial.print("I_Amp= ");
Serial.println(I_Amp);

//delay(1000);

}
