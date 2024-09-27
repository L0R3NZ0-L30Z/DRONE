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
#include <esp_system.h>



WiFiServer server(80);
Adafruit_MPU6050 mpu;
Preferences preferences;
QMC5883LCompass compass;


String incomingData;
const char* ssid = "SSID";
const char* password = "PASSWORD";
String msj;
int TimingVar = 950;
float bat = 99.9;
int DatosMagnetometro[2] = { 0, 0 };
float DatosAcelerometro[] = { 0, 0, 0, 0 };

float DatosApp[7] = { 0, 0, 0, 0, 0, 0, 0 };  //Slider, Up, Down, Left, Right,horario, anti-horario
Servo BrushlessM1;
Servo BrushlessM2;
Servo BrushlessM3;
Servo BrushlessM4;
int PW[4] = { 0, 0, 0, 0 };  // ORDEN M1,M2,M3,M4
float PWRoll;
float PWPitch;
float PWYaw;
float RpE = 0;
float PpE = 0;
float YpE = 0;
const int M1 = 15;
const int M2 = 25;
const int M3 = 23;
const int M4 = 33;
float KpRoll;
float KiRoll;
float KdRoll;
float KpPitch;
float KiPitch;
float KdPitch;
float KpYaw;
float KiYaw;
float KdYaw;
const int AnguloDeControl = 2;



void SerialUpdt() {
  Serial.println("Tiempo para actualizar valores del PID");
  Serial.println("Formato: KpRoll/KiRoll/KdRoll/KpPitch/KiPitch/KdPitch/KpYaw/KiYaw/KdYaw");
  for (int i = 0; i < 3000; i++) {
    readSerialData();
    delay(1);
  }
  Serial.println("Finalizo para actualizar valores del PID");
  printandset();
  delay(1000);
}

void WifiStart() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  server.begin();
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}
void MotorStart() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  BrushlessM1.setPeriodHertz(50);
  BrushlessM2.setPeriodHertz(50);
  BrushlessM3.setPeriodHertz(50);
  BrushlessM4.setPeriodHertz(50);

  BrushlessM1.attach(M1, 1000, 2000);
  BrushlessM2.attach(M2, 1000, 2000);
  BrushlessM3.attach(M3, 1000, 2000);
  BrushlessM4.attach(M4, 1000, 2000);
}
void MPU6050Start() {
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
void CompassStart() {
  compass.init();
  DatosMagnetometro[0] = Magnetometro();
}

void readSerialData() {
  if (Serial.available() > 0) {
    while (Serial.available()) {
      char c = Serial.read();
      incomingData += c;
      delay(2);
    }
    Serial.println("Received data:");
    Serial.println(incomingData);
    filterAndStore();
    incomingData = "";
  }
}
void filterAndStore() {
  float values[9] = { 0 };

  int index = 0;
  int startIndex = 0;
  for (int i = 0; i < incomingData.length(); i++) {
    if (incomingData[i] == '/') {
      String valueStr = incomingData.substring(startIndex, i);
      values[index++] = valueStr.toFloat();
      startIndex = i + 1;
    }
  }
  if (startIndex < incomingData.length()) {
    String valueStr = incomingData.substring(startIndex);
    values[index] = valueStr.toFloat();
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

void printandset() {
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
  float value = preferences.getFloat(key, 0.0);
  Serial.print(key);
  Serial.print(": ");
  Serial.println(value);
  preferences.end();
  return value;
}
float storeValue(const char* key, float value) {
  preferences.begin("my-app", false);
  preferences.putFloat(key, value);
  preferences.end();
  return 0;
}
void WifiConection() {
  WiFiClient client = server.available();
  if (client.available()) {
    while (client.connected()) {
      char c = client.read();
      if (c == '\n') { break; }
      msj += c;
    }
    //Serial.println(msj);
    clasify();
    msj = "";
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/plain");
    client.println("Connection: close");
    client.println();
    client.println("Ok");
    client.stop();
  }
}
void clasify() {
  /*GET /Res?ID=8896&Slider=0&XGyro=0.005&YGyro=0.015&0x=-2.6025&0y=3.82375 HTTP/1.1*/
  /*GET /Res?ID=8896&Slider=0&Up=0.005&Down=0.015&Left=-2.6025&Right=3.82375 HTTP/1.1*/
  /*GET /Res?ID=1453&Slider=0&Up=0&Down0&Left=0&Right=0&Hora=0&Anti=0 HTTP/1.1*/
  /*GET /Res?ID=6741&Slider=0&Axis=1&KP=0&KI=0&KD=0 HTTP/1.1*/

  int temp;
  String var = "";
  for (int i = 12; i <= 16; i++) var += msj.charAt(i);
  temp = var.toInt();

  if (TimingVar > 9800 && temp < 1150) {
    assign();
    TimingVar = temp;
  } else if (TimingVar < temp && temp < TimingVar + 1501) {
    assign();
    TimingVar = temp;
  }
}
void assign() {
  int Axis = 0;
  String var = "";
  int i;
  int cont = 24;
  for (i = 24; msj[i] != '&'; i++) {
    var += msj[i];
    cont++;
  }
  DatosApp[0] = var.toFloat();
  var = "";
  cont = cont + 6;
  for (i = cont; msj[i] != '&'; i++) {
    var += msj[i];
    cont++;
  }
  Axis = var.toFloat();
  var = "";

  cont = cont + 4;
  for (i = cont; msj[i] != '&'; i++) {
    var += msj[i];
    cont++;
  }
  if (Axis == 1) {
    KpRoll = var.toFloat();
  } else if (Axis == 2) {
    KpYaw = var.toFloat();
  } else if (Axis == 3) {
    KpPitch = var.toFloat();
  }
  var = "";

  cont = cont + 4;
  for (i = cont; msj[i] != '&'; i++) {
    var += msj[i];
    cont++;
  }
  if (Axis == 1) {
    KiRoll = var.toFloat();
  } else if (Axis == 2) {
    KiYaw = var.toFloat();
  } else if (Axis == 3) {
    KiPitch = var.toFloat();
  }
  var = "";

  cont = cont + 4;
  for (i = cont; msj[i] != '&'; i++) {
    var += msj[i];
    cont++;
  }
  if (Axis == 1) {
    KdRoll = var.toFloat();
  } else if (Axis == 2) {
    KdYaw = var.toFloat();
  } else if (Axis == 3) {
    KdPitch = var.toFloat();
  }
  var = "";
}
void Acelerometro() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  DatosAcelerometro[0] = a.acceleration.x;
  DatosAcelerometro[1] = a.acceleration.y;
}
int Magnetometro() {
  compass.read();
  return compass.getAzimuth();
}
void procesMag() {
  int act = Magnetometro();
  int actP = Magnetometro();
  int invAct = DatosMagnetometro[0];

  if (DatosMagnetometro[0] > 0) {
    invAct -= 180;
    act -= DatosMagnetometro[0];
    if (act < -180) {
      act = (-1) * invAct + actP + 180;
    }
    DatosMagnetometro[1] = act;
  };
  if (DatosMagnetometro[0] < 0) {
    invAct += 180;
    act += (DatosMagnetometro[0]) * -1;
    if (act > 180) {
      act = (-1) * invAct + actP - 180;
    }
    DatosMagnetometro[1] = act;
  };
  if (DatosMagnetometro[0] == 0) {
    DatosMagnetometro[1] = act;
  };
}

void PIDRoll() {
  int var;
  if (DatosApp[3] == 1) {
    var = AnguloDeControl;
  } else if (DatosApp[4] == 1) {
    var = AnguloDeControl * (-1);
  } else {
    var = 0;
  }
  float E = DatosAcelerometro[1] - var;
  float IoutRoll = IoutRoll + (E * KiRoll);
  PWRoll = (E * KpRoll) + ((E - RpE) * KdRoll) + IoutRoll;
  RpE = DatosAcelerometro[1] - var;
}
void PIDPitch() {
  int var;
  if (DatosApp[1] == 1) {
    var = AnguloDeControl;
  } else if (DatosApp[2] == 1) {
    var = AnguloDeControl * (-1);
  } else {
    var = 0;
  }
  float E = DatosAcelerometro[0] - var;
  float IoutPitch = IoutPitch + (E * KiPitch);
  PWPitch = (E * KpPitch) + ((E - PpE) * KdPitch) + IoutPitch;
  PpE = DatosAcelerometro[0] - var;
}
void PIDYaw() {
  float E = DatosMagnetometro[1];
  float IoutYaw = IoutYaw + (E * KiYaw);
  PWYaw = (E * KpYaw) + ((E - YpE) * KdYaw) + IoutYaw;
  YpE = DatosMagnetometro[0] - DatosMagnetometro[1];
}
void PIDconvert() {
  /*
  SOBRE LA POSICION DE LOS MOTORES":
  ^^  M1  M2 ^^   ^^  PW[0]  PW[1] ^^
  ^^  M3  M4 ^^   ^^  PW[2]  PW[3] ^^
  SOBRE EL GIROSCOPIO
  X= ROLL
  Y=PITCH
*/
  PW[0] = DatosApp[0] - PWRoll * (2000 / 109);
  +PWPitch*(2000 / 109) + PWYaw*(2000 / 109) * (9 / 5);
  PW[1] = DatosApp[0] + PWRoll * (2000 / 109);
  +PWPitch*(2000 / 109) - PWYaw*(2000 / 109) * (9 / 5);
  PW[2] = DatosApp[0] - PWRoll * (2000 / 109);
  -PWPitch*(2000 / 109) - PWYaw*(2000 / 109) * (9 / 5);
  PW[3] = DatosApp[0] + PWRoll * (2000 / 109);
  -PWPitch*(2000 / 109) + PWYaw*(2000 / 109) * (9 / 5);
}
void MotorDriver() {
  BrushlessM1.write(PW[0]);
  BrushlessM2.write(PW[1]);
  BrushlessM3.write(PW[2]);
  BrushlessM4.write(PW[3]);
}

void handleHttpRequestTask(void* parameter) {
    for (;;) {
        WifiConection();
        vTaskDelay(80 / portTICK_PERIOD_MS);  // Delay for 100 ms
    }
}

void sensorProcessingTask(void* parameter) {
    for (;;) {
        Acelerometro();
        procesMag(); 
        PIDRoll();
        PIDPitch();
        PIDYaw();
        PIDconvert();
        MotorDriver();
        vTaskDelay(10 / portTICK_PERIOD_MS);  // Delay for 10 ms
    }
}

void setup() {
  Serial.begin(115200);
  SerialUpdt();
  WifiStart();
  //pinMode(X, INPUT);                     //PIN A DEFINIR PARA CONTROLAR LA CARGA DE LA BATERIA
  //pinMode(2, OUTPUT);
  MotorStart();
  MPU6050Start();
  CompassStart();

  xTaskCreatePinnedToCore(handleHttpRequestTask, "HTTP Task", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(sensorProcessingTask, "Sensor Task", 4096, NULL, 1, NULL, 0);
}
void loop() {

 /*  int bri = 0;
  WifiConection();
  Acelerometro();
  procesMag();
  PIDRoll();
  PIDPitch();
  PIDYaw();
  PIDconvert();
  MotorDriver(); */
}