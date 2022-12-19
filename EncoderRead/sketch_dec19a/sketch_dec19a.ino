#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>

#define M1INA 13 //Right
#define M1INB 12
#define M1PWM 27

int interruptsTotal = 0;
volatile int interrupts_A = 0;
volatile int interrupts_B = 0;
const int encoder_A = 18;
const int encoder_B = 5;
int counter = 0;

const int freq = 80000;
const int resolution = 8;
const int pwmChannel_Right = 0;
float dutyCyclemax = 70;

long newposition;
long oldposition = 0;
unsigned long newtime;
unsigned long oldtime = 0;
float vel;

void interruptFunction_A() {
  interrupts_A += 1;
}

void interruptFunction_B() {
  interrupts_B += 1;
}


void setup() {
  Serial.begin(115200);

  pinMode(M1INA, OUTPUT);  
  pinMode(M1INB, OUTPUT);

  ledcSetup(pwmChannel_Right, freq, resolution);
  ledcAttachPin(M1PWM, pwmChannel_Right);

  attachInterrupt(encoder_A, interruptFunction_A, FALLING);
  //attachInterrupt(encoder_B, interruptFunction_B, FALLING);

  digitalWrite(M1INA, HIGH);  
  digitalWrite(M1INB, LOW);
}

void loop() {
  newposition = (float)interrupts_A;
  newtime = millis();
  vel = ((float)newposition-(float)oldposition) * 1000.0 /(((float)newtime-(float)oldtime)*12.0*47.0);
  Serial.print ("speed = ");
  Serial.println (vel);
  oldposition = newposition;
  oldtime = newtime;

  ////Serial.println((interrupts_B + interrupts_A));
  ledcWrite(pwmChannel_Right, dutyCyclemax);
  if(counter%50 == 0){
    dutyCyclemax+=1;
  }
  
  delay(5);
  counter += 1;
}
