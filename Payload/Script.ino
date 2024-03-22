//PINES DISPONIBLES 14 ,17, 19, 21, 22, 23.
/*
COMMANDS RECIEVED:
TX = Timer tick X
TY = Timer tick Y
TT = Timer tick Thumb
0X = Cero click X
0Y = Cero click Y
T  = Ready?
B  = Battery level request

SOBRE LA POSICION DE LOS MOTORES":
 ^^  M1  M2 ^^
 ^^  M3  M4 ^^
*/

#include <WiFi.h>
WiFiServer server(80);

const char* ssid = "SSID";                  //REMPLAZAR POR SSID
const char* password = "PASS";              //REMPLAZAR POR CONTRA
String header; 
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
  if(client){
    lastTime = millis();
    //Serial.println("Nuevo cliente");
    String currentLine = "";
   
    while(client.connected() && millis() - lastTime <= timeout){

      if(client.available()){
       
        char c = client.read();
        Serial.write(c);        
        header += c;
       
        if(c == '\n'){
          if(currentLine.length() == 0){
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
            if (header.indexOf("/T") >= 0) {
              String st1 = "Con@";
              String st2 = st1 + bat;
              Serial.println("Conect");
              client.println(st2);}

            if (header.indexOf("/B") >= 0) {
              String st1 = "Con@";
              String st2 = st1 + bat;
              Serial.println("Nivel de bateria");
              client.println(st2);
              }  
            /*
            if (header ) {
              temp = client.read();
              Trim(1);
              }                        
            else if (header("/01-") >= 0) {
              temp = client.read();
              Trim(1);}*/
           
            break;}
           
          else{
            currentLine = ".";
          }
        }
        else if ( c != '\r'){
          currentLine += c;
        }  
      }
    }

    header = "";
    client.stop();
  }
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
