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

const int dutyCyclemax = 190;
const int dutyCyclemin = 80;
const int dutyCycleControlFactor = dutyCyclemax/100;

const int maxDistance = 500;

int startButton;
int dutyCycle = 200;

float distance = 0.0;

int pinLidarRightFront = 16;
int pinLidarRightBack = 19;
int lidarSwitchCounter = 0; 


//Encoder Variables Right Motor
const int encoder_A_Right = 18;
long newposition_Encoder_Right;
long oldposition_Encoder_Right = 0;
unsigned long newtime_Encoder_Right;
unsigned long oldtime_Encoder_Right = 0;
float vel_tire_Right;
volatile int interrupts_A_Right = 0;

//Encoder Variables Left Motor
const int encoder_A_Left = 5;
long newposition_Encoder_Left;
long oldposition_Encoder_Left = 0;
unsigned long newtime_Encoder_Left;
unsigned long oldtime_Encoder_Left = 0;
float vel_tire_Left;
volatile int interrupts_A_Left = 0;

//PID constants
double kp = 4000.0;//30.45; //Big  80
double ki = 0.000001;//2.0;//0.02; //Small 0.002
double kd = 0.00001;
//PID variables
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;
double Setpoint = 0.0;
bool pidController = false;


//Support variables
float beta = 0;
float alpha = 0;
float carWidth = 70; //mm
float radiousCurve = 0;


WiFiClient oEspClient;
PubSubClient oClient(oEspClient);


//Interrupt Function for Encoder Count
void interruptFunction_A_Right() {
  interrupts_A_Right += 1;
}

void interruptFunction_A_Left() {
  interrupts_A_Left += 1;
}

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
    sensor[i].setTimeout(100);
    //sensor[i].setMeasurementTimingBudget(100);
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.println(" init");
  }
  digitalWrite(M1INA, HIGH);  
  digitalWrite(M1INB, LOW);
  digitalWrite(M2INA, HIGH);  
  digitalWrite(M2INB, LOW);
  


  //Encoder Interrupt Setup
  attachInterrupt(encoder_A_Right, interruptFunction_A_Right, FALLING);
  attachInterrupt(encoder_A_Left, interruptFunction_A_Left, FALLING);

}

void loop() {
  static unsigned long ulNextTime=0+5; 

  //For MQTT Message
  char acMsg[100];


  //Lidar Measurment

  for (int i = 0; i < sensorCount; i++) {
      distances[i] = sensor[i].readRangeSingleMillimeters();
      if (distances[i] == 65535) distances[i] = -1; //failed to measure distance
      if (distances[i] >= 8190) distances[i] = 600;   
      

  }
  distanceRightBack = distances[2];
  distanceRight = distances[0];
  distanceLeft = distances[1];
  Serial.println(distanceLeft);
  //}  
  headerAngle = atan2((float)(distanceRightBack-distanceRight), 70.0)*180.0/3.1415; 
  //Serial.println("Header Angle: "); Serial.println(headerAngle);
  

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

  float pmwFast = dutyCyclemax + abs(distance)/(float)maxDistance*70*timeMultiplier*(dutyCycleControlFactor); // 500*70*1*0 = Kp = 3500
  float pmwSlow = dutyCyclemax -abs(distance)/(float)maxDistance*30*timeMultiplier*sq(dutyCycleControlFactor);

  if (pmwFast > 255){
    pmwFast = 255;
  }
  if (pmwSlow < 60){
    pmwSlow = 60;
  }

  //Control DIY
  if (pidController){
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
  }else{
    input = distance;
    //Serial.print("PID Input ");Serial.println(input); 
    output = computePID(input);  
    Serial.print("PID Output ");Serial.println(output); 
    Serial.println("");
    if(output<0){
      pmwLeft = dutyCyclemax + abs(output);
      pmwRight = dutyCyclemax - abs(output);

      //VLeft and VRight Relation
      //radiousCurve = 70/sin(headerAngle*3.1415/180);
      //pmwLeft = pmwRight* (radiousCurve + carWidth)/radiousCurve;
    }else{
      pmwLeft = dutyCyclemax - abs(output);
      pmwRight = dutyCyclemax + abs(output);

      //VLeft and VRight Relation
      //radiousCurve = 70/sin(headerAngle*3.1415/180);
      //pmwRight = pmwLeft* (radiousCurve + carWidth)/radiousCurve;
    }
  }

  


  
  ledcWrite(pwmChannel_Right, pmwRight);
  ledcWrite(pwmChannel_Left, pmwLeft);
  //Serial.print("PMW Right");Serial.println(pmwRight);
  //Serial.print("PMW Left");Serial.println(pmwLeft);
  




  //Calculate Tire Speed with Encoder Data
  long ulTime= millis();
  if(ulTime>=ulNextTime)
  {
    newposition_Encoder_Right = interrupts_A_Right;
    newposition_Encoder_Left = interrupts_A_Left;
    newtime_Encoder_Right = millis();
    newtime_Encoder_Left = millis();
    vel_tire_Right = ((float)newposition_Encoder_Right-(float)oldposition_Encoder_Right) * 1000.0 /(((float)newtime_Encoder_Right-(float)oldtime_Encoder_Right)*12.0*47.0);
    vel_tire_Left = ((float)newposition_Encoder_Left-(float)oldposition_Encoder_Left) * 1000.0 /(((float)newtime_Encoder_Left-(float)oldtime_Encoder_Left)*12.0*47.0);

    vel_tire_Right=vel_tire_Right*2*3.1415*0.035;
    vel_tire_Left=vel_tire_Left*2*3.1415*0.035;
    //Serial.print ("speed Right= ");
    //Serial.println (vel_tire_Right*2*3.1415*0.035);
    //Serial.print ("speed Left= ");
    //Serial.println (vel_tire_Left*2*3.1415*0.035);
    oldposition_Encoder_Right = newposition_Encoder_Right;
    oldtime_Encoder_Right = newtime_Encoder_Right;
    oldposition_Encoder_Left = newposition_Encoder_Left;
    oldtime_Encoder_Left = newtime_Encoder_Left;
    ulNextTime += 5;
  }
  
  sprintf(acMsg,"%f", headerAngle);
  oClient.publish("Niklas/HeaderAngle", &acMsg[0]);
  //sprintf(acMsg,"%f", timeMultiplier);
  //oClient.publish("Niklas/TimerMultiplier", &acMsg[0]);
  sprintf(acMsg,"%f", vel_tire_Left);
  oClient.publish("Niklas/VLeft", &acMsg[0]);
  sprintf(acMsg,"%f", vel_tire_Right);
  oClient.publish("Niklas/VRight", &acMsg[0]);
  //sprintf(acMsg,"%f",radiousCurve);
  sprintf(acMsg,"%f", distance);
  oClient.publish("Niklas/Distance",&acMsg[0]);
  sprintf(acMsg,"%f", pmwRight);
  oClient.publish("Niklas/PmwRight",&acMsg[0]);
  sprintf(acMsg,"%f", pmwLeft);
  oClient.publish("Niklas/PmwLeft",&acMsg[0]);
  oClient.loop();
}


double computePID(double inp){     
        currentTime = millis();                //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
        
        error = Setpoint - inp;                                // determine error
        cumError += error * elapsedTime;                // compute integral
        rateError = (error - lastError)/elapsedTime;   // compute derivative

        double out = kp*error + ki*cumError + kd*rateError;                //PID output               

        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time

        
        return out;                                        //have function return the PID output
}
