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
*/

#include <WiFi.h>
WiFiServer server(80);

const char* ssid = "SSID";//REMPLAZAR POR SSID
const char* password = "PASS";//REMPLAZAR POR CONTRA
String header; 
float bat = 99.9;
float Giroscopio[3] ={0,0,0};
float GiroscopioApp[3] ={0,0,0};
unsigned long lastTime, timeout = 2000;
#define M1 12
#define M2 14
#define M3 26
#define M4 27

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
  pinMode(12,OUTPUT);
  pinMode(14,OUTPUT);
  pinMode(26,OUTPUT);
  pinMode(27,OUTPUT);  
  //VVVV COMPLETAMENTE OPCIONAL VVVV
  tone(12,1975, 250);
  tone(14,1975, 250);
  tone(26,1975, 250);
  tone(27,1975, 250);
  delay(350);
  tone(12,2349, 250);
  tone(14,2349, 250);
  tone(26,2349, 250);
  tone(27,2349, 250);
  delay(350);
  tone(12,2637, 250);
  tone(14,2637, 250);
  tone(26,2637, 250);
  tone(27,2637, 250);
  delay(350);
  tone(12,1975, 400);
  tone(14,1975, 400);
  tone(26,1975, 400);
  tone(27,1975, 400);
  delay(420);
  tone(12,2637, 400);
  tone(14,2637, 400);
  tone(26,2637, 400);
  tone(27,2637, 400);
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
void MotorDriver(){

}

void setup() {

  Serial.begin(115200);
  WifiStart();//INICIO DE RECEPCION DE DATOS
  MotorStart();
  //pinMode(,INPUT);//PIN A DEFINIR PARA CONTROLAR LA CARGA DE LA BATERIA
 
}

void loop() {//NO PONER DELAYS!!!!!!!
  WifiConection();//RECEPCION DE DATOS
  //Giro();
  //PIDRoll();
  //PIDPitch();
  //PIDYaw();
  //PIDAdder
  MotorDriver();
}
