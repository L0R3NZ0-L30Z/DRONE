


void setup() {

  Serial.begin(115200);
}

//compass.setSamplingRate(10);
//-----------↑↑↑↑↑------------QMC5883L Setup-----------↑↑↑↑↑------------




void loop() {
  //int r;
  //int16_t x,y,z,t;
 
  int heading = compass.readHeading();
  //r = compass.readRaw(&x,&y,&z,&t);
 
  Serial.print("Degree: ");
  Serial.println(heading);
  //Serial.print("| Rotation: ");
  //Serial.println(r);
 
  //delay(250);
}
