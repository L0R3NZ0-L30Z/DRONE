#include <WiFi.h>
//PINES DISPONIBLES 14 ,17, 19, 21, 22, 23
/*
COMMANDS RECIEVED:
TX = Timer tick X
TY = Timer tick Y
TT = Timer tick Thumb
0X = Cero click X
0Y = Cero click Y
T  = Ready?
B  = Batterie level request
*/
const char* ssid = "SSID";
const char* password = "PASS";

WiFiServer server(80);

float bat = 99.9;
float Giroscopio[3] ={0,0,0};
float GiroscopioApp[3] ={0,0,0};
String header;

unsigned long lastTime, timeout = 2000;

void Inicio_Wifi(){
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
void Coneccion_Wifi(){
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

void setup() {

  Serial.begin(115200);
  Inicio_Wifi();//INICIO DE RECEPCION DE DATOS
 
}

void loop() {//NO PONER DELAYS!!!!!!!
  Coneccion_Wifi();//RECEPCION DE DATOS
 

}
