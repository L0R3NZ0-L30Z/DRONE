#include <ESP32Servo.h>
#include "WiFi.h"

WiFiServer server(80);

const char* ssid = "SSID";
const char* password = "PASSWORD";
String msj;
int TimingVar = 950;
float bat = 99.9;

int DatosMagnetometro[] = {0, 0, 0, 0};
float DatosAcelerometro[] = {0, 0, 0, 0};
float DatosApp[5] = {0, 0, 0, 0, 0};

Servo BrushlessM1;
Servo BrushlessM2;
Servo BrushlessM3;
Servo BrushlessM4;

int PW[4] = {0, 0, 0, 0};
float PWRoll;
float PWPitch;
float PWYaw;
float RpE = 0;
float PpE = 0;
float YpE = 0;

#define M1 12
#define M2 14
#define M3 26
#define M4 27

#define KpRoll   0
#define KiRoll   0
#define KdRoll   0

#define KpPitch   0
#define KiPitch   0
#define KdPitch   0

#define KpYaw   0
#define KiYaw   0
#define KdYaw   0

#define PRDiv 1

void WifiStart(){
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  server.begin();
}

void MotorStart(){
  BrushlessM1.attach(M1, 1000, 2000); 
  BrushlessM2.attach(M2, 1000, 2000); 
  BrushlessM3.attach(M3, 1000, 2000); 
  BrushlessM4.attach(M4, 1000, 2000); 
}

void WifiConection(){
  WiFiClient client = server.available();
    if(client.available()){
      while(client.connected()){
       
        char c = client.read();
        if(c == '\n'){break;}
          msj += c;
        }
        clasify();
        msj="";
  }
}

void clasify(){
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
 
  cont = cont + 7;
  for(i=cont; msj[i]!='&'; i++){var += msj[i];cont++;}
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
}

void PIDRoll(){
  float E = DatosAcelerometro[0] - DatosApp[1];
  float IoutRoll = IoutRoll + (E * KiRoll);
  PWRoll = (E * KpRoll) + ((E - RpE) * KdRoll) + IoutRoll;
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
  PW[0] = DatosApp[0] + PWRoll * (2000/109) + PWPitch * (2000/109) + PWYaw * (2000/109);
  PW[1] = DatosApp[0] - PWRoll * (2000/109) + PWPitch * (2000/109) - PWYaw * (2000/109);
  PW[2] = DatosApp[0] + PWRoll * (2000/109) - PWPitch * (2000/109) - PWYaw * (2000/109);
  PW[3] = DatosApp[0] - PWRoll * (2000/109) - PWPitch * (2000/109) + PWYaw * (2000/109);
}

void MotorDriver(){ 
  BrushlessM1.write(PW[0]);
  BrushlessM2.write(PW[1]);
  BrushlessM3.write(PW[2]);
  BrushlessM4.write(PW[3]);
 }

void setup() {
  Serial.begin(115200);
  WifiStart();
  pinMode(32, OUTPUT);
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  BrushlessM1.setPeriodHertz(50);
  BrushlessM2.setPeriodHertz(50);  
  BrushlessM3.setPeriodHertz(50);  
  BrushlessM4.setPeriodHertz(50); 
  MotorStart();
}

void loop() {
  int bri = 0;
  WifiConection();
  PIDRoll();
  PIDPitch();
  PIDYaw();
  PIDconvert();
  MotorDriver();
  bri = DatosApp[5] * 17 / 12;
  analogWrite(32, bri);
}
