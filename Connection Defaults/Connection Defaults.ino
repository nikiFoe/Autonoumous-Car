#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <string.h>


void setup(){
  Serial.begin(115200);
  Serial.print(__TIME__);
  Serial.print("  ");
  connectWifi();
  PubSubClient oClient(oEspClient) = connectMqtt();
  subscribeMQTT("Niklas/");
}

void loop(){
  Serial.println("Boop");
  delay(1000);
 oClient.loop();
}