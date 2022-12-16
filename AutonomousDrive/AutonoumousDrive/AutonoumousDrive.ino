#include <Wire.h>
#include <VL53L0X.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <math.h>
#include <Wire.h>
#include <VL53L0XMod.h>
#include "SlowSoftI2CMaster.h"

#define HIGH_SPEED#
VL53L0X sensor_R;
VL53L0X sensor_L;

VL53L0XMod sensor[3];
int distances[3];
int sensorCount = 3;


#define M1INA 13 //Right
#define M1INB 12
#define M1PWM 27

#define M2INA 14 // Left
#define M2INB 15
#define M2PWM 32

//MQTT and WIFI
const char* pcSSID = "iPSK-UMU"; // Enter your WiFi name
const char* pcPASSWORD =  "HejHopp!!"; // Enter WiFi password
const char* pcMqttServer = "tfe.iotwan.se";
const int   iMqttPort = 1883;
const char* pcMqttUser = "intro22";
const char* pcMqttPassword = "outro";


// Setting PWM properties
const int freq = 80000;
const int pwmChannel_Right = 0;
const int pwmChannel_Left = 1;
const int resolution = 8;
int distanceRight = 0;
int distanceRightBack = 0;
int distanceLeft = 0;
float headerAngle = 0.0;
int lastDistance = 0;
float timeMultiplier = 1.0;
long lastTimeNormal = 0;

const int dutyCyclemax = 100;
const int dutyCyclemin = 80;
const int dutyCycleControlFactor = dutyCyclemax/100;

const int maxDistance = 500;

int startButton;
int dutyCycle = 200;

float distance = 0.0;

int pinLidarRightFront = 16;
int pinLidarRightBack = 19;
int lidarSwitchCounter = 0; 

WiFiClient oEspClient;
PubSubClient oClient(oEspClient);

//MQTT Receive Callback
void fcMqttCallback(char* pcTopic, byte* pcPayload, unsigned int iLength)
{
  char acMsg[100];
  Serial.print("\nMessage arrived in topic: ");
  Serial.println(pcTopic);
  String topic = String(pcTopic);
  Serial.print("Received MQTT message:");
  Serial.println((char*)pcPayload);

  if(topic == "Niklas/StartButton"){
    startButton = atoi((char*)pcPayload);
    for (int i=0;i<iLength;i++){
      //Serial.print((topic)); Serial.println((char)pcPayload[i]);
      //startButton = ((char*)pcPayload)[i] - '0';
      //sprintf(acMsg,"%i\n", startButton);

      Serial.print(" (Remote button): "); Serial.println(startButton);
    }
  }
}

//Wifi Connection
void connectToWifi(){
  WiFi.begin(pcSSID,pcPASSWORD);
  while (WiFi.status() != WL_CONNECTED){
    delay(2000);
    Serial.print("Connecting to WiFi SSID:");
    Serial.println(pcSSID);
  }
  Serial.println("Connected to the WiFi network.");
  
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  oClient.setServer( pcMqttServer, iMqttPort);
  oClient.setCallback(fcMqttCallback);
  while (!oClient.connected()) {
    Serial.print("Connecting to MQTT...");
    Serial.print(pcMqttServer);
    if (oClient.connect("NiklasMainESP32", pcMqttUser, pcMqttPassword )) {
      Serial.println(".  Connection established."); 
    } 
    else {
      Serial.print(". failed to connect with state ");
      Serial.println(oClient.state());
      delay(2000);
    }
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(M1INA, OUTPUT);  
  pinMode(M1INB, OUTPUT); 
  //pinMode(M1PWM, OUTPUT); 

  pinMode(M2INA, OUTPUT); 
  pinMode(M2INB, OUTPUT); 
  //pinMode(M2PWM, OUTPUT); 


  //Connection Wifi and MQTT
  //oClient.subscribe("Niklas/TimerMultiplier");
  connectToWifi();  
  oClient.subscribe("Niklas/StartButton");
  // configure LED PWM functionalitites
  ledcSetup(pwmChannel_Right, freq, resolution);
  ledcSetup(pwmChannel_Left, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(M1PWM, pwmChannel_Right);
  ledcAttachPin(M2PWM, pwmChannel_Left);

  sensor[0].setI2CPin(22, 23); // Sensor Right Front
  sensor[1].setI2CPin(17, 21); // Sensor Left
  sensor[2].setI2CPin(16, 19); // Sensor Right Back

   for (int i = 0; i < sensorCount; i++) {
    sensor[i].init();
    sensor[i].setTimeout(50);
    sensor[i].setMeasurementTimingBudget(100);
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.println(" init");
  }
  digitalWrite(M1INA, HIGH);  
  digitalWrite(M1INB, LOW);
  digitalWrite(M2INA, HIGH);  
  digitalWrite(M2INB, LOW);
}

void loop() {
  //For MQTT Message
  char acMsg[100];


  //Lidar Measurment

  for (int i = 0; i < sensorCount; i++) {
      distances[i] = sensor[i].readRangeSingleMillimeters();
      if (distances[i] == 65535) distances[i] = -1; //failed to measure distance
      if (distances[i] >= 8190) distances[i] = 0;   
  }

  distanceRightBack = distances[2];
  distanceRight = distances[0];
  distanceLeft = distances[1];
  
  //}  
  headerAngle = atan2((float)(distanceRightBack-distanceRight), 70.0)*180.0/3.1415; 
  Serial.print("Header Angle: "); Serial.println(headerAngle);
  

  //Serial.print("Distance (mm): "); Serial.println(distance);
  if(distanceRight > maxDistance){
    distanceRight = maxDistance;
  }

  if(distanceLeft > maxDistance){
    distanceLeft = maxDistance;
  }
  distance = (float)distanceRight - (float)distanceLeft - 5.0;
  //Serial.println(distance);

  //Serial.print("Distance: "); Serial.println((abs(lastDistance) - abs(distance)));
  long currentTime = millis();
  if ((abs(distance) - abs(lastDistance) ) > 100.0){
    timeMultiplier = ((float)currentTime - (float)lastTimeNormal)/1000.0;
  }else{
    lastTimeNormal = currentTime;
    timeMultiplier = 1.0;
    lastDistance = distance;
  }
  


  //Serial.print("Time Multipl: "); Serial.println(timeMultiplier);
  float pmwRight;
  float pmwLeft;

  float pmwFast = dutyCyclemax + abs(distance)/(float)maxDistance*70*timeMultiplier*(dutyCycleControlFactor);
  float pmwSlow = dutyCyclemax -abs(distance)/(float)maxDistance*30*timeMultiplier*sq(dutyCycleControlFactor);

  if (pmwFast > 255){
    pmwFast = 255;
  }
  if (pmwSlow < 60){
    pmwSlow = 60;
  }


  if (true){
    //Serial.println("Motor On");
    if(distance>= -2.0 && distance<=2.0){
      pmwRight = dutyCyclemax;
      pmwLeft = dutyCyclemax;
    }else if(distance < -2.0){
      pmwRight = pmwFast;
      pmwLeft = pmwSlow;
    } else if (distance > 2.0){
      pmwLeft = pmwFast;
      pmwRight = pmwSlow;
    }
  }else{
    pmwRight = 0;
    pmwLeft = 0;
  }
  ledcWrite(pwmChannel_Right, pmwRight);
  ledcWrite(pwmChannel_Left, pmwLeft);
  //Serial.print("PMW Right");Serial.println(pmwRight);
  //Serial.print("PMW Left");Serial.println(pmwLeft);

  sprintf(acMsg,"%f", headerAngle);
  oClient.publish("Niklas/HeaderAngle", &acMsg[0]);
  sprintf(acMsg,"%f", timeMultiplier);
  oClient.publish("Niklas/TimerMultiplier", &acMsg[0]);
  sprintf(acMsg,"%f", distance);
  oClient.publish("Niklas/Distance",&acMsg[0]);
  sprintf(acMsg,"%f", pmwRight);
  oClient.publish("Niklas/PmwRight",&acMsg[0]);
  sprintf(acMsg,"%f", pmwLeft);
  oClient.publish("Niklas/PmwLeft",&acMsg[0]);
  oClient.loop();
}
