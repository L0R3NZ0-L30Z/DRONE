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
#include <WiFi.h>
WiFiServer server(80);

const char* ssid = "Iphone 5000";                  //REMPLAZAR POR SSID
const char* password = "12345678";              //REMPLAZAR POR CONTRA
String msj;                                 //STRING QUE GUARDA EL MENSAJE RECIBIDO POR WIFI
float bat = 99.9;                           //VARIABLE DE ALAMACENAMIENTO DE NIVEL DE BATERIA
                                            //FALTA VARIABLE DE MAGNETOMETRO!
float Giroscopio[2] = {0,0};                //VARIABLE DE POSICION DE GIROSCOPIO DEL DRON // X, Y
float DatosApp[3] = {0,0,5};                //VARIABLE DE DATOS DE DIRECCION Y POTENCIA DE LA APP
float DatT[10]={0};


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
    case 'P':
      msj.remove(0, 6);
      DatosApp[0] = msj.toFloat();
    case 'R':
      msj.remove(0, 6);
      DatosApp[1] = msj.toFloat();
    case 'T':
      msj.remove(0, 6);
      //ProcessT(msj.toFloat());
  }
}/*
void ProcessT(float var){
  for(int i=9; i>=0; i++){
    DatT[i]=DatT[i-1];
  }
  DatT[0]= var;
  for(int o=0; o<=10; o++){
    DatosApp[2] = DatT[o];}
  DatosApp[2] =  DatosApp[2] / 10;
  }*/
  
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
  Serial.println(DatosApp[2]);
  delay(20);                                //UNICO DELAY PARA DEJA PROCESAR
}
