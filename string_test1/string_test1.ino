void setup() {

static char mydata[50] = {};
float distance= 56235;

}

void loop() {

  char dist1[]= "distance= "; 
  char uint16_t[]= (distance/10); 
  char dist3[]= " cm";
  strcat(dist1, dist2);
  strcat(dist1, dist3);
  mydata= dist1;
  serial.print(mydata);
  delay(3000);
                   

}
