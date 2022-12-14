#include "Adafruit_VL53L0X.h"
#include <Wire.h>


#define M1INA 13 //Right
#define M1INB 12
#define M1PWM 27

#define M2INA 14 // Left
#define M2INB 15
#define M2PWM 32

Adafruit_VL53L0X lox_right = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_left = Adafruit_VL53L0X();

// Setting PWM properties
const int freq = 80000;
const int pwmChannel_Right = 0;
const int pwmChannel_Left = 1;
const int resolution = 8;
int distanceRight = 0;

const int dutyCyclemax = 100;
const int dutyCyclemin = 80;

const int maxDistance = 400;

int distanceLeft = 0;
int dutyCycle = 200;

float distance = 0.0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(M1INA, OUTPUT);  
  pinMode(M1INB, OUTPUT); 
  //pinMode(M1PWM, OUTPUT); 

  pinMode(M2INA, OUTPUT); 
  pinMode(M2INB, OUTPUT); 
  //pinMode(M2PWM, OUTPUT); 
  //Wire1.begin(23, 22);

  // configure LED PWM functionalitites
  ledcSetup(pwmChannel_Right, freq, resolution);
  ledcSetup(pwmChannel_Left, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(M1PWM, pwmChannel_Right);
  ledcAttachPin(M2PWM, pwmChannel_Left);

  Wire1.begin(23, 22);
  //Gyro
  Wire.begin(21, 17);

  // attach the channel to the GPIO to be controlled
  //ledcAttachPin(M1PWM, PWMA_Channel);

  if (!lox_right.begin(0x29, false, &Wire1))
  {
    Serial.println(F("Failed to boot VL53L0X Right"));
    while(1); 
  }

  if (!lox_left.begin(0x29, false, &Wire))
  {
    Serial.println(F("Failed to boot VL53L0X Left"));
    while(1); 
  }

  digitalWrite(M1INA, HIGH);  
  digitalWrite(M1INB, LOW);
  digitalWrite(M2INA, HIGH);  
  digitalWrite(M2INB, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  static unsigned long ulNextTime=0+10000; 
  VL53L0X_RangingMeasurementData_t measure_right;
  VL53L0X_RangingMeasurementData_t measure_left;

  lox_right.rangingTest(&measure_right, false); // pass in 'true' to get debug data printout!
  lox_left.rangingTest(&measure_left, false);
  


    
  if (measure_right.RangeStatus != 4)
  { // phase failures have incorrect data
    distanceRight = measure_right.RangeMilliMeter;
    Serial.print("Distance Right (mm): "); Serial.println(distanceRight);
    Serial.println("");
  }
  if (measure_left.RangeStatus != 4)
  { // phase failures have incorrect data
    distanceLeft = measure_left.RangeMilliMeter;
    Serial.print("Distance Left(mm): "); Serial.println(distanceLeft);
    Serial.println("");
  }

  //Serial.print("Distance (mm): "); Serial.println(distance);
  if(distanceRight > maxDistance){
    distanceRight = maxDistance;
  }

  if(distanceLeft > maxDistance){
    distanceLeft = maxDistance;
  }
  distance = (float)distanceRight - (float)distanceLeft;
  Serial.println(distance);

  if(distance <= -10){
    ledcWrite(pwmChannel_Right, dutyCyclemax + abs(distance)*abs(distance)/(float)maxDistance);
    ledcWrite(pwmChannel_Left, dutyCyclemax - abs(distance)*abs(distance)/(float)maxDistance*0.5);
    Serial.print("PMW Right");Serial.println(dutyCyclemax + abs(distance)*abs(distance)/(float)maxDistance);
    Serial.print("PMW Left");Serial.println(dutyCyclemax - abs(distance)*abs(distance)/(float)maxDistance*0.5);
    delay(10);    

  } else if (distance > 10){
    Serial.println(dutyCyclemax + dutyCyclemax*(float)distanceLeft/(float)distanceRight);
    ledcWrite(pwmChannel_Left, dutyCyclemax + abs(distance)*abs(distance)/(float)maxDistance ); 
    ledcWrite(pwmChannel_Right, dutyCyclemax - abs(distance)*abs(distance)/(float)maxDistance*0.5);
    Serial.print("PMW Right");Serial.println(dutyCyclemax - abs(distance)*abs(distance)/(float)maxDistance);
    Serial.print("PMW Left");Serial.println(dutyCyclemax + abs(distance)*abs(distance)/(float)maxDistance*0.5);
    delay(10);

  }

}
