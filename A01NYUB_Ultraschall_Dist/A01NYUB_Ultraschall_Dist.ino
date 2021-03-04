/*
  AnalogReadSerial

  Reads an analog input on pin 0, prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/AnalogReadSerial
*/

// the setup routine runs once when you press reset:
#define mess_TX A0 // Grove A0
#define mess_RX // Grove A1
const int pinSIG = A0;
const int pinNC = A1;

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
  //pinMode(pinSIG, INPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin A1:
  int sensorValue = analogRead(A0);
  // print out the value you read:
  Serial.println(sensorValue);
  delay(1);        // delay in between reads for stability
}
