//PINES DISPONIBLES 14 ,17, 19, 21, 22, 23.

#include <WiFi.h>
WiFiServer server(80);

const char* ssid = "SSID";                  //REMPLAZAR POR SSID
const char* password = "PASS";              //REMPLAZAR POR CONTRA
String msj; 
float bat = 99.9;                           //VARIABLE DE ALAMACENAMIENTO DE NIVEL DE BATERIA
                                            //FALTA VARIABLE DE MAGNETOMETRO!
float Giroscopio[3] ={0,0,0};               //VARIABLE DE POSICION DE GIROSCOPIO DEL DRON
float GiroscopioApp[3] ={0,0,0};            //VARIABLE DE POSICION DE GIROSCOPIO DE LA APP
unsigned long lastTime, timeout = 2000;
int PW[4] = {0,0,0,0};                      //VARIABLES FINALES DEL DUTY CICLE DEL PWM DE LOS MOTORES // ORDEN M1,M2,M3,M4
int PWRoll[4] = {0,0,0,0};                  //VARIABLES DE ROLL 
int PWPitch[4] = {0,0,0,0};                 //VARIABLES DE PITCH
int PWYaw[4] = {0,0,0,0};                   //VARIABLES DE YAW
#define M1 12                               //DEFINICION DE MOTORES
#define M2 14                               //DEFINICION DE MOTORES
#define M3 26                               //DEFINICION DE MOTORES
#define M4 27                               //DEFINICION DE MOTORES
#define KpRoll   0                          //DEFINICION DE CONSTANTES PARA AJUSTE PID
#define KiRoll   0                          //DEFINICION DE CONSTANTES PARA AJUSTE PID
#define KdRoll   0                          //DEFINICION DE CONSTANTES PARA AJUSTE PID
#define KpPitch   0                         //DEFINICION DE CONSTANTES PARA AJUSTE PID
#define KiPitch   0                         //DEFINICION DE CONSTANTES PARA AJUSTE PID
#define KdPitch   0                         //DEFINICION DE CONSTANTES PARA AJUSTE PID
#define KpYaw   0                           //DEFINICION DE CONSTANTES PARA AJUSTE PID
#define KiYaw   0                           //DEFINICION DE CONSTANTES PARA AJUSTE PID
#define KdYaw   0                           //DEFINICION DE CONSTANTES PARA AJUSTE PID


void WifiStart(){
  //Serial.print("Conectando a ");
  //Serial.println(ssid);
  WiFi.begin(ssid,password);
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");}
  Serial.println("Direccion IP: ");
  Serial.print(WiFi.localIP());
  server.begin();
}
void clasify(){
  int len = msj.length() - 10;
  msj.remove(len, 9);
  switch(msj[5]){
    case 'X':
      msj.remove(0, 6);
      GiroscopioApp[0] = msj.toFloat();
      
    case 'Y':
      msj.remove(0, 6);
      GiroscopioApp[1] = msj.toFloat();
    case 'T':
      msj.remove(0, 6);
      GiroscopioApp[2] = msj.toFloat();
    case 'F':
      msj.remove(0, 6);
      GiroscopioApp[3] = msj.toFloat();
    case 'W':
      msj.remove(0, 6);
      GiroscopioApp[4] = msj.toFloat();  
  }
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
 
    /*
  COMMANDS RECIEVED:
  X = X
  Y = Y
  T = Thumb
  F = Cero click X
  W = Cero click Y
  T  = Ready?
  B  = Battery level request

  SOBRE LA POSICION DE LOS MOTORES":
  ^^  M1  M2 ^^
  ^^  M3  M4 ^^
  */
  WiFiClient client = server.available();
  //client.println(bat);
  //for(int i=0; i5; i++){
    if(client.available()){
      while(client.connected()){
        char c = client.read();
        msj += c;
        if(c == '\n'){break;}
        }
        //Serial.println(msj);
        clasify();
        client.println(".");
        msj="";
        delay(1);  
      }
  //}

}
void PIDAdder(){
  PW[0] = (PWRoll[0] + PWPitch[0] + PWYaw[0])/3;  
  PW[1] = (PWRoll[1] + PWPitch[1] + PWYaw[1])/3;
  PW[2] = (PWRoll[2] + PWPitch[2] + PWYaw[2])/3;
  PW[3] = (PWRoll[3] + PWPitch[3] + PWYaw[3])/3;
}
void MotorDriver(){ 
  analogWrite(M1, PW[0]);
  analogWrite(M2, PW[1]);
  analogWrite(M3, PW[2]);
  analogWrite(M4, PW[3]);
}


void setup() {

  Serial.begin(115200);
  WifiStart();                              //INICIO DE RECEPCION DE DATOS
  MotorStart();
  //pinMode(X, INPUT);                      //PIN A DEFINIR PARA CONTROLAR LA CARGA DE LA BATERIA
 
}

void loop() {                               //NO PONER DELAYS!!!!!!!
  WifiConection();                          //RECEPCION DE DATOS
  //Giro();                                 //INPUT DEL GIROSCOPIO
  //Magnetometro();                         //INPUT DEL MAGNETOMETRO
  //PIDRoll();                              //PID ROLL
  //PIDPitch();                             //PID PITCH
  //PIDYaw();                               //PID YAW
  PIDAdder();                               //SUMA DE LOS OUTPUT DE LOS PID
  MotorDriver();
  delay(2);                                //UNICO DELAY PARA DEJA PROCESAR
}
