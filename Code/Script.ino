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
#include <WiFi.h>
//#include "I2Cdev.h"
//#include "MPU6050.h"
#include "Wire.h"
WiFiServer server(80);

const char* ssid = "SSID";                  //REMPLAZAR POR SSID
const char* password = "PASSWORD";              //REMPLAZAR POR CONTRA
String msj;                                 //STRING QUE GUARDA EL MENSAJE RECIBIDO POR WIFI
int TimingVar=950;
float bat = 99.9;                           //VARIABLE DE ALAMACENAMIENTO DE NIVEL DE BATERIA
                                            //FALTA VARIABLE DE MAGNETOMETRO!
float AnglePitchX;
float AnglePitchY;
float ax, ay;
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
#define M1 12                               //DEFINICION DE MOTORES
#define M2 14                               //DEFINICION DE MOTORES
#define M3 26                               //DEFINICION DE MOTOREaS
#define M4 27                               //DEFINICION DE MOTORES
#define KpRoll   0                          //DEFINICION DE CONSTANTE PARA AJUSTE PID
#define KiRoll   0                          //DEFINICION DE CONSTANTE PARA AJUSTE PID
#define KdRoll   0                          //DEFINICION DE CONSTANTE PARA AJUSTE PID
#define KpPitch   0                         //DEFINICION DE CONSTANTE PARA AJUSTE PID
#define KiPitch   0                         //DEFINICION DE CONSTANTE PARA AJUSTE PID
#define KdPitch   0                         //DEFINICION DE CONSTANTE PARA AJUSTE PID
#define KpYaw   0                           //DEFINICION DE CONSTANTE PARA AJUSTE PID
#define KiYaw   0                           //DEFINICION DE CONSTANTE PARA AJUSTE PID
#define KdYaw   0                           //DEFINICION DE CONSTANTE PARA AJUSTE PID
#define PRDiv 1                             //DEFINICION DE CONSTANTE PARA DIVIDIR LA ENTRADA DE LA POTENCIA
#define ConstProportional 25.992883792      //DEFINICION DE CONSTANTE PARA AJUSTAR LA SALIDA DEL PID

void WifiStart(){
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println(".");
  }
  server.begin();
}
void assign(){
  String var = "";
  int i;
  int cont=24;
  for(i=24; msj[i]!='&'; i++){var += msj[i];cont++;}
  DatosApp[0]= var.toFloat();
  for (int i = 0; i < 4; ++i) {
    PW[i] = DatosApp[0];
    }
  var = "";

  cont = cont + 7;
  for(i=cont; msj[i]!='&'; i++){var += msj[i];cont++;}
  DatosApp[1]= var.toFloat();
  var = "";
  
  cont = cont + 7;
  for(i=cont; msj[i]!='&'; i++){var += msj[i];cont++;}
  DatosApp[2]= var.toFloat();
  var = "";
  
  cont = cont + 4;
  for(i=cont; msj[i]!='&'; i++){var += msj[i];cont++;}
  DatosApp[2]= var.toFloat();
  var = "";
  
  cont = cont + 4;
  for(i=cont; msj[i]!='\0'; i++){var += msj[i];cont++;}
  
  DatosApp[2]= var.toFloat();
  var = "";

  Serial.print("Slider: "); Serial.print(DatosApp[0]); Serial.print("  ");
  Serial.print("Gyro X Axis: "); Serial.print(DatosApp[1]); Serial.print("  "); 
  Serial.print("Gyro Y Axis: "); Serial.print(DatosApp[2]); Serial.print("  "); 
  Serial.print("Zero X Axis: "); Serial.print(DatosApp[3]); Serial.print("  "); 
  Serial.print("Zero Y Axis: "); Serial.print(DatosApp[4]); Serial.print("  "); 
  Serial.println("uT");
  }
void clasify(){
  /*GET /Res?ID=1205&Slider=165.75&XGyro=0.04875&YGyro=0.04875&0x=-0.06&0y=0.015 HTTP/1.1*/
  int temp;
  String var = "";
  for(int i=12; i!='&'; i++){var += msj[i];}
  temp = var.toInt();
  if(TimingVar<temp && temp<TimingVar + 8000){
    assign();
    TimingVar=temp;
  }
  else if(TimingVar>9950 && temp<1050){
    assign();
    TimingVar=temp;
  } 
}
void MotorStart(){
  BrushlessM1.attach(M1, 1000, 2000); 
  BrushlessM2.attach(M2, 1000, 2000); 
  BrushlessM3.attach(M3, 1000, 2000); 
  BrushlessM4.attach(M4, 1000, 2000); 
}
void WifiConection(){
  WiFiClient client = server.available();
  client.println("GET /Res?Slider=xx&XGyro=xx&YGyro=xx&0x=xx&0y=xx HTTP/1.1");
  client.println("Host: " + String(WiFi.localIP()));
  client.println("Connection: close");
  client.println();
  msj = client.readStringUntil('\n');
  //Serial.println(msj);
  clasify();
}
void MotorDriver(){ 
  BrushlessM1.write(PW[0]);
  BrushlessM2.write(PW[1]);
  BrushlessM3.write(PW[2]);
  BrushlessM4.write(PW[3]);
 }
 /*
void PIDRoll(){
  float E = Giroscopio[0] - DatosApp[0];
  float IoutRoll = IoutRoll + (E * KiRoll);
  PWRoll = (E * KpRoll) + ((E - RpE) * KdRoll) + IoutRoll;
  RpE = Giroscopio[0] - DatosApp[0];
}
void PIDPitch(){
  float E = Giroscopio[1] - DatosApp[1];
  float IoutPitch = IoutPitch + (E * KiPitch);
  PWPitch = (E * KpPitch) + ((E - PpE) * KdPitch) + IoutPitch;
  PpE = Giroscopio[1] - DatosApp[1];
}
/*void PIDYaw(){
  float E = Magnetometro;1
  float IoutYaw = Ioutyaw + (E * KiYaw);
  PWYaw = (E * KpYaw) + ((E - YpE) * KdYaw) + IoutYaw;
  YpE = Magnetometro;
}*/
void setup() {
  Serial.begin(115200);
  WifiStart();                              //INICIO DE RECEPCION DE DATOS
  //incialicia_Gyro();
  //pinMode(X, INPUT);                     //PIN A DEFINIR PARA CONTROLAR LA CARGA DE LA BATERIA
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);    // standard 50 hz servo
  MotorStart();
 
}

void loop() {                               //NO PONER DELAYS!!!!!!!
  WifiConection();                          //RECEPCION DE DATOS
  //Giro();                                 //INPUT DEL GIROSCOPIO
  //Magnetometro();                         //INPUT DEL MAGNETOMETRO
  //PIDRoll();                              //PID ROLL
  //PIDPitch();                             //PID PITCH
  //PIDYaw();                               //PID YAW
  //PIDconvert();                           //SUMA DE LOS OUTPUT DE LOS PID
  MotorDriver();
  Serial.println(PW[0]);
  //delay(20);                                //UNICO DELAY PARA DEJA PROCESAR
}
