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
  }
  server.begin();
}
void assign(){
  String var = "";
  int i;
  int cont=24;
  for(i=24; msj[i]!='&'; i++){var += msj[i];cont++;}
  Serial.print("Slider: "); Serial.print(var); Serial.print("  ");
  DatosApp[0]= var.toFloat();
  var = "";

  cont = cont + 7;
  for(i=cont; msj[i]!='&'; i++){var += msj[i];cont++;}
  Serial.print("Gyro X Axis: "); Serial.print(var); Serial.print("  "); 
  DatosApp[1]= var.toFloat();
  var = "";
  
  cont = cont + 7;
  for(i=cont; msj[i]!='&'; i++){var += msj[i];cont++;}
  Serial.print("Gyro Y Axis: "); Serial.print(var); Serial.print("  "); 
  DatosApp[2]= var.toFloat();
  var = "";
  
  cont = cont + 4;
  for(i=cont; msj[i]!='&'; i++){var += msj[i];cont++;}
  Serial.print("Zero Y Axis: "); Serial.print(var); Serial.print("  "); 
  DatosApp[2]= var.toFloat();
  var = "";
  
  cont = cont + 4;
  for(i=cont; msj[i]!='\0'; i++){var += msj[i];cont++;}
  Serial.print("Zero Y Axis: "); Serial.print(var); Serial.print("  "); 
  Serial.println("uT");
  DatosApp[2]= var.toFloat();
  var = "";
  
  }
void clasify(){
  /*GET /Res?ID=1205&Slider=165.75&XGyro=0.04875&YGyro=0.04875&0x=-0.06&0y=0.015 HTTP/1.1*/
  int temp;
  String var = "";
  for(int i=12; i!='&'; i++){var += msj[i];}
  temp = var.toInt();
  if(TimingVar<temp && temp<TimingVar + 100){
    Serial.println(var);
    assign();
    TimingVar=temp;
  }
  else if(TimingVar>9950 && temp<1050){
    Serial.println(var);
    assign();
    TimingVar=temp;
  } 
}
void Sensor(float &ax, float &ay){
  // Get new sensor events with the readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
 
  // Print out the values
  /*
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
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
  Serial.print(AnglePitchX);
  Serial.println(" °c");
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

  ax = a.acceleration.x;
  ay = a.acceleration.y;
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
  client.println("GET /Res?Slider=xx&XGyro=xx&YGyro=xx&0x=xx&0y=xx HTTP/1.1");
  client.println("Host: " + String(WiFi.localIP()));
  client.println("Connection: close");
  client.println();
  msj = client.readStringUntil('\n');
  //Serial.println(msj);
  clasify();
}
void MotorDriver(){ 
  for (int i = 0; i < 4; ++i) {
    analogWrite(M1 + i, PW[i]);
    }
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
  //A CHEKEAR
  /*SOBRE LA POSICION DE LOS MOTORES":
  ^^  M1  M2 ^^     ^^ PW[0]  PW[1] ^^
  ^^  M3  M4 ^^     ^^ PW[2]  PW[3] ^^ */
  if(PWRoll >= 0){
    PW[0] = PWRoll * ConstProportional;
    PW[2] = PWRoll * ConstProportional;
  }
  else if(PWRoll < 0){
    PW[1] = PWRoll * ConstProportional;
    PW[3] = PWRoll * ConstProportional;
  }
  if(PWPitch >= 0){
    PW[0] = ((PWRoll * ConstProportional) + PW[0]) / 2;
    PW[1] = ((PWRoll * ConstProportional) + PW[1]) / 2;
  }
  else if(PWPitch < 0){
    PW[2] = ((PWRoll * ConstProportional) + PW[2]) / 2;
    PW[3] = ((PWRoll * ConstProportional) + PW[3]) / 2;
  }
  else if(PWYaw >= 0){
    PW[0] = ((PWRoll * ConstProportional) + PW[0]) / 3;
    PW[3] = ((PWRoll * ConstProportional) + PW[3]) / 3;
  }
  else if(PWYaw < 0){
    PW[1] = ((PWRoll * ConstProportional) + PW[1]) / 3;
    PW[2] = ((PWRoll * ConstProportional) + PW[2]) / 3;
  }

  PW[0] = (PW[0] + (DatosApp[3] / PRDiv)) / 2; 
  PW[1] = (PW[1] + (DatosApp[3] / PRDiv)) / 2; 
  PW[2] = (PW[2] + (DatosApp[3] / PRDiv)) / 2; 
  PW[3] = (PW[3] + (DatosApp[3] / PRDiv)) / 2; 
}
/*void incialicia_Gyro(){


    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);

    // initialize device
    /Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    //Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // use the code below to change accel/gyro offset values
    
    accelgyro.setXGyroOffset(220);
    accelgyro.setYGyroOffset(76);
    accelgyro.setZGyroOffset(-85);
    //Serial.println("Updating internal sensor offsets...");
    // -76 -2359 1688 0 0 0
    //Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    //Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    //Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    //Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    //Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    //Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    //Serial.print("\n");
    
    

    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);
}
void lectura(){
     // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
        //Serial.print("a/g:\t");
        //Serial.print(ax); Serial.print("\t");
        //Serial.print(ay); Serial.print("\t");
        //Serial.print(az); Serial.print("\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        //Serial.println(gz);
    #endif
   // blink LED to indicate activity
    //blinkState = !blinkState;
    //digitalWrite(LED_PIN, blinkState);
    }-*/

void setup() {
  Serial.begin(115200);
  WifiStart();                              //INICIO DE RECEPCION DE DATOS
  //incialicia_Gyro();
  //MotorStart();
  //pinMode(X, INPUT);                     //PIN A DEFINIR PARA CONTROLAR LA CARGA DE LA BATERIA

    if (!mpu.begin()) {
    //Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  //Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  /*Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
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
    break;
  case MPU6050_RANGE_500_DEG:
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
    break;
  case MPU6050_BAND_184_HZ:
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

void loop() {                               //NO PONER DELAYS!!!!!!!
  WifiConection();                          //RECEPCION DE DATOS
  //Giro();                                 //INPUT DEL GIROSCOPIO
  //Magnetometro();                         //INPUT DEL MAGNETOMETRO
  /*PIDRoll();                              //PID ROLL
  PIDPitch();                             //PID PITCH
  PIDYaw();                               //PID YAW
  PIDconvert();                           //SUMA DE LOS OUTPUT DE LOS PID
  MotorDriver();*/
  //Serial.println(DatosApp[0]);
  delay(20);                                //UNICO DELAY PARA DEJA PROCESAR
}
