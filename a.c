//-----------↓↓↓↓↓------------MPU6050 Libs-----------↓↓↓↓↓------------

//-----------↑↑↑↑↑------------MPU6050 Libs-----------↑↑↑↑↑------------




//-----------↓↓↓↓↓------------MPU6050 Setup-----------↓↓↓↓↓------------




float ax, ay;




void loop() {

  Sensor(ax, ay);

  Serial.print("Acceleration X: ");
  Serial.print(ax);
  Serial.print(", Y: ");
  Serial.println(ay);*/
  //delay(500);
}
