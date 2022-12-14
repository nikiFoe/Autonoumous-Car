#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <string.h>


//Wifi Setup 
const char* pcSSID = "iPSK-UMU"; // Enter your WiFi name
const char* pcPASSWORD =  "HejHopp!!"; // Enter WiFi password

//MQTT Setup
const char* pcMqttServer = "tfe.iotwan.se";
const int   iMqttPort = 1883;
const char* pcMqttUser = "intro22";
const char* pcMqttPassword = "outro";

WiFiClient oEspClient;
PubSubClient oClient(oEspClient);


// Connection to Wifi established
void connectWifi(){
  WiFi.begin(pcSSID,pcPASSWORD);
  while (WiFi.status() != WL_CONNECTED){
    delay(2000);
    Serial.print("Connecting to WiFi SSID:");
    Serial.println(pcSSID);
  }
}


//Connection to MQTT established
PubSubClient connectMqtt() {
  oClient.setServer( pcMqttServer, iMqttPort);
  oClient.setCallback(fcMqttCallback);
  return oClient;
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

void subscribeMQTT(char* topic){
  Serial.print(topic);
  oClient.subscribe(topic);
}

void subscribeMQTT(char* topic, char* Msg){
  oClient.publish(topic, Msg);
}

void fcMqttCallback(char* pcTopic, byte* pcPayload, unsigned int iLength)
{
  Serial.print("Niklas");
  char* topic = pcTopic;
  char* payload = (char*)pcPayload;
  
 
}