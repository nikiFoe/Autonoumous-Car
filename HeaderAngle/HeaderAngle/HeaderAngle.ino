#include <Wire.h>
#include <VL53L0X.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <math.h>

#define HIGH_SPEED#
VL53L0X sensor_Front;
VL53L0X sensor_Back;

int distanceFront = 0;
int distanceBack = 0;

float headerAngle = 0.0;

void setup() {
  Serial.begin(115200);

  Wire1.begin(23,22);
  Wire2.begin(20,19);
  Wire.begin(21, 17);

  sensor_Front.setBus(&Wire);
  sensor_Back.setBus(&Wire1);

  sensor_Front.setTimeout(500);
  if (!sensor_Front.init())
  {
    Serial.println("Failed to detect and initialize sensor L!");
    while (1) {}
  }

  sensor_Back.setTimeout(500);
  if (!sensor_Back.init())
  {
    Serial.println("Failed to detect and initialize sensor R!");
    while (1) {}
  }

  sensor_Front.setMeasurementTimingBudget(100000);
  sensor_Back.setMeasurementTimingBudget(100000);
  sensor_Front.startContinuous();
  sensor_Back.startContinuous();

}

void loop() {
  distanceFront = sensor_Front.readRangeContinuousMillimeters() + 4;
  Serial.print("Distance Front (mm): "); Serial.println(distanceFront);
  //Serial.println("");

  distanceBack = sensor_Back.readRangeContinuousMillimeters();
  Serial.print("Distance Back(mm): "); Serial.println(distanceBack);
  Serial.println("");

  headerAngle =   atan2 ((distanceBack-distanceFront),142);
  Serial.print("Header Angle: "); Serial.println(headerAngle*180/3.1415);

}
