//PINES DISPONIBLES 14 ,17, 19, 21, 22, 23.
/*
  SOBRE LA POSICION DE LOS MOTORES":
  ^^  M1  M2 ^^
  ^^  M3  M4 ^^
  SOBRE EL GIROSCOPIO
  X= ROLL
  Y=PITCH
*/

/*
Advertencias:
- Tener en cuenta que la posicion del giroscopio/*accelerometro y magnetometro hara que haya que 
  redefinir la configuracion de los motores y demas varables.

*/
/*
  INCLUIR:
  WiFi.RSSI()

  WiFi.setHostname(YOUR_NEW_HOSTNAME);

  IPAddress local_IP(192, 168, 1, 184);
  // Set your Gateway IP address
  IPAddress gateway(192, 168, 1, 1);

  IPAddress subnet(255, 255, 0, 0);
  IPAddress primaryDNS(8, 8, 8, 8);   // optional
  IPAddress secondaryDNS(8, 8, 4, 4); // optional
*/
#include <ESP32Servo.h>
#include "WiFi.h"
#include <Adafruit_MPU6050.h>
#include <QMC5883LCompass.h>
#include <I2Cdev.h>
#include <Wire.h>
#include <math.h>
#include <SPI.h>
#include <Arduino.h>
#include <Preferences.h>


WiFiServer server(80);
Adafruit_MPU6050 mpu;
Preferences preferences;
QMC5883LCompass compass;


String incomingData;
const char* ssid = "SSID";                  //REMPLAZAR POR SSIDconst          
const char* password = "PASSWORD";          //REMPLAZAR POR CONTRA
String msj;                                 //STRING QUE GUARDA EL MENSAJE RECIBIDO POR WIFi
int TimingVar=950;
float bat = 99.9;                           //VARIABLE DE ALAMACENAMIENTO DE NIVEL DE BATERIAnt 
int DatosMagnetometro[] = {0,0,0,0};             //dato,dato,origen,origen                                
float DatosAcelerometro[] = {0,0,0,0};           //dato,dato,origen,origen

float DatosApp[5] = {0,0,0,0,0};                //VARIABLE DE DATOS DE DIRECCION Y POTENCIA DE LA APP
Servo BrushlessM1;
Servo BrushlessM2;
Servo BrushlessM3;
Servo BrushlessM4;
int PW[4] = {0,0,0,0};                      //VARIABLES FINALES DEL DUTY CICLE DEL PWM DE LOS MOTORES // ORDEN M1,M2,M3,M4
float PWRoll;                               //VARIABLES DE ROLL 
float PWPitch;                              //VARIABLES DE PITCH
float PWYaw;                                //VARIABLES DE YAW
float RpE = 0;
float PpE = 0;
float YpE = 0;
const int M1 = 12;                               //DEFINICION DE MOTORES
const int M2 = 14;                               //DEFINICION DE MOTORES
const int M3 = 26;                               //DEFINICION DE MOTOREaS
const int M4 = 27;                               //DEFINICION DE MOTORES 
float KpRoll;
float KiRoll;
float KdRoll;
float KpPitch;
float KiPitch;
float KdPitch;
float KpYaw;
float KiYaw;
float KdYaw;


void WifiStart(){
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("."); }
  server.begin();
}
void MotorStart(){
  BrushlessM1.attach(M1, 1000, 2000); 
  BrushlessM2.attach(M2, 1000, 2000); 
  BrushlessM3.attach(M3, 1000, 2000); 
  BrushlessM4.attach(M4, 1000, 2000); 
}
void MPU6050Start(){
  while (!Serial)
    delay(10);
  if (!mpu.begin()) {
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  /*Serial.print("Accelerometer range set to: ");

  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break; case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }*/
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  /*Serial.print("Gyro range set to: ");

  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break; case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }*/
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  /*Serial.print("Filter bandwidth set to: ");

  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break; case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
  Serial.println("");*/
  //delay(100);
}

void readSerialData() {
    if (Serial.available() > 0) {
        while (Serial.available()) {
            char c = Serial.read();           
            incomingData += c;
            delay(2);}
        Serial.println("Received data:");
        Serial.println(incomingData);
        filterAndStore();
        incomingData = "";
    }
}
void filterAndStore() {
    int values[9] = {0};

    int index = 0;
    int startIndex = 0;
    for (int i = 0; i < incomingData.length(); i++) {
        if (incomingData[i] == '/') {           
          String valueStr = incomingData.substring(startIndex, i);
          values[index++] = valueStr.toInt();
          startIndex = i + 1;
          }   
        }
    if (startIndex < incomingData.length()) {
        String valueStr = incomingData.substring(startIndex);
        values[index] = valueStr.toInt();
    }
    storeValue("KpRoll", values[0]);
    storeValue("KiRoll", values[1]);    
    storeValue("KdRoll", values[2]);
    storeValue("KpPitch", values[3]);
    storeValue("KiPitch", values[4]);
    storeValue("KdPitch", values[5]);   
    storeValue("KpYaw", values[6]);
    storeValue("KiYaw", values[7]);
    storeValue("KdYaw", values[8]);
    printandset();
}

void printandset(){
  KpRoll = printvalue("KpRoll");
  KiRoll = printvalue("KiRoll");
  KdRoll = printvalue("KdRoll"); 
  KpPitch = printvalue("KpPitch");
  KiPitch = printvalue("KiPitch");
  KdPitch = printvalue("KdPitch");
  KpYaw = printvalue("KpYaw");
  KiYaw = printvalue("KiYaw");
  KdYaw = printvalue("KdYaw");
}
float printvalue(const char* key) {
    preferences.begin("my-app", false);
    float value = preferences.getInt(key, 0);
    Serial.print(key);
    Serial.print(": ");
    Serial.println(value);   
    preferences.end();
    return value;
}
void storeValue(const char* key, int value) {
  preferences.begin("my-app", false);
  preferences.putInt(key, value);
  preferences.end();
}
void WifiConection(){ WiFiClient client = server.available();
    if(client.available()){
      while(client.connected()){
        char c = client.read();       
        if(c == '\n'){break;}
          msj += c;
        }
        //Serial.println(msj);
        clasify();
        msj="";
  }
}
void clasify(){
  /*GET /Res?ID=8896&Slider=0&XGyro=0.005&YGyro=0.015&0x=-2.6025&0y=3.82375 HTTP/1.1*/
  int temp;
  String var = "";
  for(int i=12; i<=16; i++) var += msj.charAt(i);
  temp = var.toInt();
 
  if(TimingVar>9800 && temp<1150){
    assign();
    TimingVar=temp;
  }
  else if(TimingVar<temp && temp<TimingVar + 1501){
    assign();
    TimingVar=temp;
  }
   
}
void assign(){
  String var = "";
  int i;
  int cont=24;
  for(i=24; msj[i]!='&'; i++){var += msj[i];cont++;}
  DatosApp[0]= var.toFloat();
  var = "";
  cont = cont + 7;
  for(i=cont; msj[i]!='&'; i++){var += msj[i];cont++;}

  DatosApp[1]= var.toFloat();
  var = "";
 
  cont = cont + 7; for(i=cont; msj[i]!='&'; i++){var += msj[i];cont++;}
  DatosApp[2]= var.toFloat();
  var = "";
 
  cont = cont + 4;
  for(i=cont; msj[i]!='&'; i++){var += msj[i];cont++;}
  DatosApp[3]= var.toFloat();
  var = "";
 
  cont = cont + 4;
  for(i=cont; msj[i]!='\0'; i++){var += msj[i];cont++;}
  DatosApp[4]= var.toFloat();
  var = "";
  /*
  Serial.print("Slider: "); Serial.print(DatosApp[0]); Serial.print("  ");
  Serial.print("Gyro X Axis: "); Serial.print(DatosApp[1]); Serial.print("  ");
  Serial.print("Gyro Y Axis: "); Serial.print(DatosApp[2]); Serial.print("  ");
  Serial.print("Zero X Axis: "); Serial.print(DatosApp[3]); Serial.print("  ");
  Serial.print("Zero Y Axis: "); Serial.print(DatosApp[4]); Serial.print("  ");
  Serial.println("uT");*/
}
void Acelerometro(){
  //float AnglePitchX;    //Variables para tener angulos en grados
  //float AnglePitchY;    //Variables para tener angulos en grados
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  /*
  Serial.print("Acceleration X: ");

  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: "); Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");
  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");
  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");*/
  //AnglePitchX=-atan(a.acceleration.x/sqrt(a.acceleration.y*a.acceleration.y+a.acceleration.z*
  //a.acceleration.z))*1/(3.142/180);

  // Print out the values
  /*
  Serial.print("Angle X: ");
  Serial.print(AnglePitchX); Serial.println(" °c");
  */
  //AnglePitchY=-atan(a.acceleration.y/sqrt(a.acceleration.x*a.acceleration.x+a.acceleration.z*
  //a.acceleration.z))*1/(3.142/180);
  // Print out the values
  /*
  Serial.print("Angle Y: ");
  Serial.print(AnglePitchY);
  Serial.println(" °c");
 
  Serial.println("");*/
  //delay(500);
  DatosAcelerometro[0] = a.acceleration.x;
  DatosAcelerometro[1] = a.acceleration.y;
  
  
}
void Magnetometro(){
  compass.read();
  DatosMagnetometro[1] = compass.getAzimuth();
  if(DatosMagnetometro[1]<0){DatosMagnetometro[1] +=360;}
}
void PIDRoll(){
  float E = DatosAcelerometro[0] - DatosApp[1];
  float IoutRoll = IoutRoll + (E * KiRoll); PWRoll = (E * KpRoll) + ((E - RpE) * KdRoll) + IoutRoll;
  RpE = DatosAcelerometro[0] - DatosApp[1];
}
void PIDPitch(){
  float E = DatosAcelerometro[1] - DatosApp[2];
  float IoutPitch = IoutPitch + (E * KiPitch);
  PWPitch = (E * KpPitch) + ((E - PpE) * KdPitch) + IoutPitch;
  PpE = DatosAcelerometro[1] - DatosApp[2];
}
void PIDYaw(){
  float E = DatosMagnetometro[0] - DatosMagnetometro[1];
  float IoutYaw = IoutYaw + (E * KiYaw);
  PWYaw = (E * KpYaw) + ((E - YpE) * KdYaw) + IoutYaw;
  YpE = DatosMagnetometro[0] - DatosMagnetometro[1];
}
void PIDconvert(){
  /*
  SOBRE LA POSICION DE LOS MOTORES":
  ^^  M1  M2 ^^   ^^  PW[0]  PW[1] ^^
  ^^  M3  M4 ^^   ^^  PW[2]  PW[3] ^^
  SOBRE EL GIROSCOPIO
  X= ROLL
  Y=PITCH
*/
  PW[0] = DatosApp[0] + PWRoll * (2000/109) + PWPitch * (2000/109) + PWYaw * (2000/109) * (9/5);
  PW[1] = DatosApp[0] - PWRoll * (2000/109) + PWPitch * (2000/109) - PWYaw * (2000/109) * (9/5);
  PW[2] = DatosApp[0] + PWRoll * (2000/109) - PWPitch * (2000/109) - PWYaw * (2000/109) * (9/5);
  PW[3] = DatosApp[0] - PWRoll * (2000/109) - PWPitch * (2000/109) + PWYaw * (2000/109) * (9/5);
}
void MotorDriver(){ 
  BrushlessM1.write(PW[0]);
  BrushlessM2.write(PW[1]);
  BrushlessM3.write(PW[2]);
  BrushlessM4.write(PW[3]);
 }
void setup() {
  Serial.begin(115200);

  WifiStart();                              //INICIO DE RECEPCION DE DATOS
  //pinMode(X, INPUT);                     //PIN A DEFINIR PARA CONTROLAR LA CARGA DE LA BATERIA
  pinMode(32, OUTPUT);
  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  BrushlessM1.setPeriodHertz(50);    // standard 50 hz servo 
  BrushlessM2.setPeriodHertz(50);  
  BrushlessM3.setPeriodHertz(50);  
  BrushlessM4.setPeriodHertz(50); 
  MotorStart();
  MPU6050Start();
  compass.init();
  //Medir pos accell y magne para despus
  Serial.println("Tiempo para actualizar valores del PID");
  Serial.println("Formato: KpRoll/KiRoll/KdRoll/KpPitch/KiPitch/KdPitch/KpYaw/KiYaw/KdYaw");
  for(int i=0; i<2000; i++){
    readSerialData();
    delay(1);
  }
  Serial.println("Finalizo para actualizar valores del PID");
  printandset();
}
void loop() {                               //NO PONER DELAYS!!!!!!!
  int bri=0;
  WifiConection();                          //RECEPCION DE DATOS
  Acelerometro();                                 //INPUT DEL GIROSCOPIO
  Magnetometro();                         //INPUT DEL MAGNETOMETRO PIDRoll();                              //PID ROLL
  PIDPitch();                             //PID PITCH
  PIDYaw();                               //PID YAW PID
  //convert();                             //SUMA DE LOS OUTPUT DE LOS PID
  MotorDriver();
  bri = DatosApp[5] * (17 / 12);
  analogWrite(32, bri);
  //delay(20);                                //UNICO DELAY PARA DEJA PROCESAR

  Serial.print("Slider: "); Serial.print(DatosApp[0]); Serial.print(",");
  Serial.print("Gyro X Axis: "); Serial.print(DatosApp[1]); Serial.print(",");
  Serial.print("Gyro Y Axis: "); Serial.print(DatosApp[2]); Serial.print(",");
  Serial.print("Zero X Axis: "); Serial.print(DatosApp[3]); Serial.print(",");
  Serial.print("Zero Y Axis: "); Serial.print(DatosApp[4]); Serial.print(",");
  Serial.print("Compass angle: ");Serial.print(DatosMagnetometro[1]);Serial.print(",");
  Serial.print("Accel X-Axis");Serial.print(DatosAcelerometro[0]);Serial.print(",");
  Serial.print("Accel Y-Axis");Serial.print(DatosAcelerometro[1]);Serial.print(",");
  Serial.println("uT");
}