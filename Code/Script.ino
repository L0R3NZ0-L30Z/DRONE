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
#include <WiFi.h>
WiFiServer server(80);

const char* ssid = "SSID";                  //REMPLAZAR POR SSID
const char* password = "PASSWORD";              //REMPLAZAR POR CONTRA
String msj;                                 //STRING QUE GUARDA EL MENSAJE RECIBIDO POR WIFI
float bat = 99.9;                           //VARIABLE DE ALAMACENAMIENTO DE NIVEL DE BATERIA
                                            //FALTA VARIABLE DE MAGNETOMETRO!
float Giroscopio[2] = {0,0};                //VARIABLE DE POSICION DE GIROSCOPIO DEL DRON // X, Y
float DatosApp[5] = {0,0,0,0,0};                //VARIABLE DE DATOS DE DIRECCION Y POTENCIA DE LA APP


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
void WifiStart(){
  //Serial.print("Conectando a ");
  //Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid,password);
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
    }
  Serial.println("Direccion IP: ");
  Serial.print(WiFi.localIP());
  server.begin();
}
void clasify(){
  /*/Res?Slider=xx&XGyro=xx&YGyro=xx&0x=xx&0y=xx*/
  String var = "";
  int a = msj.indexOf("Slider=");
  int b = msj.indexOf("&XGyro=");
  for(int i=a+7; i<=b; i++){var += msj[i];}
  DatosApp[0] = var.toFloat();
  var = "";
  a = msj.indexOf("&XGyro=");
  b = msj.indexOf("&YGyro=");
  for(int i=a+7; i<=b; i++){var += msj[i];}
  DatosApp[1] = var.toFloat();
  var = "";
  a = msj.indexOf("&YGyro=");
  b = msj.indexOf("&0x=");
  for(int i=a+7; i<=b; i++){var += msj[i];}
  DatosApp[2] = var.toFloat();
  var = "";
  a = msj.indexOf("&0x=");
  b = msj.indexOf("&0y=");
  for(int i=a+3; i<=b; i++){var += msj[i];}
  DatosApp[3] = var.toFloat();
  var = "";
  a = msj.indexOf("&0y=");
  b = msj.indexOf("\0");
  for(int i=a+3; i<=b; i++){var += msj[i];}
  DatosApp[4] = var.toFloat();
  var = "";
}
  
void MotorStart(){
  pinMode(M1,OUTPUT);
  pinMode(M2,OUTPUT);
  pinMode(M3,OUTPUT);
  pinMode(M4,OUTPUT);  
  //VVVV COMPLETAMENTE OPCIONAL VVVV
  tone(M1,1975, 250);
  tone(M2,1975, 250);
  tone(M3,1975, 250);
  tone(M4,1975, 250);
  delay(350);
  tone(M1,2349, 250);
  tone(M2,2349, 250);
  tone(M3,2349, 250);
  tone(27,2349, 250);
  delay(350);
  tone(M1,2637, 250);
  tone(M2,2637, 250);
  tone(M3,2637, 250);
  tone(M4,2637, 250);
  delay(350);
  tone(M1,1975, 400);
  tone(M2,1975, 400);
  tone(M3,1975, 400);
  tone(M4,1975, 400);
  delay(420);
  tone(M1,2637, 400);
  tone(M2,2637, 400);
  tone(M3,2637, 400);
  tone(M4,2637, 400);
  delay(500);
  // ^^ TERMINA OPCIONAL ^^
}
void WifiConection(){
  WiFiClient client = server.available();
  //client.println("GET /T HTTP/1.1\n\n")
  client.println("GET /Res?Slider=xx&XGyro=xx&YGyro=xx&0x=xx&0y=xx HTTP/1.1");
  client.println("Host: " + String(WiFi.localIP()));
  client.println("Connection: close");
  client.println();
  msj="";
  while(client.available()){
    char c = client.read();
    msj += c;
    }
    clasify();
}
void MotorDriver(){ 
  analogWrite(M1, PW[0]);
  analogWrite(M2, PW[1]);
  analogWrite(M3, PW[2]);
  analogWrite(M4, PW[3]);
}
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
void PIDconvert(){
  /*SOBRE LA POSICION DE LOS MOTORES":
  ^^  M1  M2 ^^     ^^ PW[0]  PW[1] ^^
  ^^  M3  M4 ^^     ^^ PW[2]  PW[3] ^^ */
  if(PWRoll >= 0){
    PW[0] = PWRoll * 25.5;
    PW[2] = PWRoll * 25.5;
  }
  else if(PWRoll < 0){
    PW[1] = PWRoll * 25.5;
    PW[3] = PWRoll * 25.5;
  }
  if(PWPitch >= 0){
    PW[0] = ((PWRoll * 25.5) + PW[0]) / 2;
    PW[1] = ((PWRoll * 25.5) + PW[1]) / 2;
  }
  else if(PWPitch < 0){
    PW[2] = ((PWRoll * 25.5) + PW[2]) / 2;
    PW[3] = ((PWRoll * 25.5) + PW[3]) / 2;
  }
  else if(PWYaw >= 0){
    PW[0] = ((PWRoll * 25.5) + PW[0]) / 3;
    PW[3] = ((PWRoll * 25.5) + PW[3]) / 3;
  }
  else if(PWYaw < 0){
    PW[1] = ((PWRoll * 25.5) + PW[1]) / 3;
    PW[2] = ((PWRoll * 25.5) + PW[2]) / 3;
  }

  PW[0] = (PW[0] + (DatosApp[3] / PRDiv)) / 2; 
  PW[1] = (PW[1] + (DatosApp[3] / PRDiv)) / 2; 
  PW[2] = (PW[2] + (DatosApp[3] / PRDiv)) / 2; 
  PW[3] = (PW[3] + (DatosApp[3] / PRDiv)) / 2; 
}


void setup() {
  Serial.begin(115200);
  WifiStart();                              //INICIO DE RECEPCION DE DATOS
  //MotorStart();
  //pinMode(X, INPUT);                      //PIN A DEFINIR PARA CONTROLAR LA CARGA DE LA BATERIA}
  
}

void loop() {                               //NO PONER DELAYS!!!!!!!
  WifiConection();                          //RECEPCION DE DATOS
  //Giro();                                 //INPUT DEL GIROSCOPIO
  //Magnetometro();                         //INPUT DEL MAGNETOMETRO
  /*PIDRoll();                              //PID ROLL
  PIDPitch();                             //PID PITCH
  PIDYaw();                               //PID YAW
  PIDconvert();                           //SUMA DE LOS OUTPUT DE LOS PID
  MotorDriver();*/
  Serial.println(DatosApp[0]);
  delay(20);                                //UNICO DELAY PARA DEJA PROCESAR
}
