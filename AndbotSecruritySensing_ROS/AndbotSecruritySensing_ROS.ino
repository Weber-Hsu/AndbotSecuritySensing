#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/RelativeHumidity.h>
#include <std_msgs/Bool.h>
#include <stdio.h>

/* Sensor libraries */
#include <DHT22.h>      // temperauture sensor

/* MEGA 2560 Pin configuration */
#define DHT22_PIN    22    // DHT22 (temperture & humidity reading) digital input
#define PIR_PIN      23    // PIR (motion sensor) digital input
#define Flame_PIN    A0    // Flame sensor V2 analog input
#define MQ2_PIN      A1    // MQ2 (smoke sensor) analog input
#define MQ9_PIN_AI   A2    // MQ9 (smoke sensor) analog input
//#define MQ9_PIN_DI   24    // MQ9 (smoke sensor) diginal input
#define Dust_PIN_AI  A3    // Sharp Optical Dust sensor analog input
#define Dust_PIN_DO  25    // Sharp Optical Dust sensor diginal

/* Setup variables used in this code*/
DHT22 andbotDHT22(DHT22_PIN);

long publisher_timer = 0;

sensor_msgs::Temperature DHT22_Temperature_msgs; // DHT22 -temperture digital input
sensor_msgs::RelativeHumidity DHT22_Humidity_msgs; // DHT22 -Humidity digital input
std_msgs::Bool PIR_msgs; //PIR (motion sensor) digital input
std_msgs::Float32 Flame_msgs; // Flame sensor V2 analog input
std_msgs::Float32 MQ2_msgs; // MQ2 (smoke sensor) analog input
std_msgs::Float32 MQ9_msgs_AI; // MQ9 (smoke sensor) analog input
//std_msgs::Bool MQ9_msgs_DI; // MQ9 (smoke sensor) digital input
std_msgs::Float32 Dust_msgs; // Sharp Optical Dust sensor analog input
std_msgs::Float32 Dust_msgs_VoMeasured; // Sharp Optical Dust sensor analog input
std_msgs::String SensorStatus_msgs; // Report back each sensors status

/*  define  ROS node and topics */
ros::NodeHandle Security;
ros::Publisher pub_DHT22Temp("/CurTemperature",&DHT22_Temperature_msgs);
ros::Publisher pub_DHT22Humid("/CurHumidity",&DHT22_Humidity_msgs);
ros::Publisher pub_PIRstate("/MotionDetection",&PIR_msgs);
ros::Publisher pub_Flame("/FlameDetection", &Flame_msgs);
ros::Publisher pub_MQ2Smoke("/MQ2", & MQ2_msgs);
ros::Publisher pub_MQ9Smoke_AI("/MQ9", & MQ9_msgs_AI);
//ros::Publisher pub_MQ9Smoke_DI("/SmokeDetectionMQ9_D", & MQ9_msgs_DI);
ros::Publisher pub_Dust("/DustDetection", & Dust_msgs);
ros::Publisher pub_Dust_V("/DustDetectionV", & Dust_msgs_VoMeasured);


/* temporary varibles for Dust sensing*/
float voMeasured = 0;
float calcVoltage = 0;
/* sampling timing of output pulse in Dust sensor (from Datasheet)*/
int samplingTime = 280; // LED Pulse Width = samplingTime + deltaTime = 320us
int deltaTime = 40;
int sleepTime = 9680; // period (per pulse) = 10ms, i.e, sleepingTime = 10ms - 320us = 9680 us

int warmup = 2000 ; //(msec)
bool SensorReadyFlag = false;

void setup() {
  //initialize mega 2560 for sensor input
  pinMode(PIR_PIN, INPUT); //setup pin
  //pinMode(MQ9_PIN_DI, INPUT);
  pinMode(Dust_PIN_DO, OUTPUT);

  /* ROS Node configurations */
  Security.initNode();
  Security.advertise(pub_DHT22Temp);
  Security.advertise(pub_DHT22Humid);
  Security.advertise(pub_PIRstate);
  Security.advertise(pub_Flame);
  Security.advertise(pub_MQ2Smoke);
  Security.advertise(pub_MQ9Smoke_AI);
  //Security.advertise(pub_MQ9Smoke_DI);
  Security.advertise(pub_Dust);
  Security.advertise(pub_Dust_V);
  

  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:

  //warmup sequence
  if (SensorReadyFlag == false)
  {
    if (millis() < warmup)
    {
      SensorReadyFlag = false;
      Serial.println("Please wait ...");
    }
    else
    {
      SensorReadyFlag = true;
      Serial.println("warmup finish");
    }
  }
  else;

  /* DHT22 reading... */
  DHT22_ERROR_t errorCode;

  if((millis() > publisher_timer) && SensorReadyFlag == true)
  {
    errorCode = andbotDHT22.readData();
    switch (errorCode)
    {
      case DHT_ERROR_NONE:
        DHT22_Temperature_msgs.temperature = (double)andbotDHT22.getTemperatureC();
        DHT22_Humidity_msgs.relative_humidity = (double)andbotDHT22.getHumidity();
        pub_DHT22Temp.publish(&DHT22_Temperature_msgs);
        Serial.println(andbotDHT22.getHumidityInt());
        pub_DHT22Humid.publish(&DHT22_Humidity_msgs);
        break;
      case DHT_ERROR_CHECKSUM:
        Serial.println("sum error");
        DHT22_Temperature_msgs.temperature = -1;
        DHT22_Humidity_msgs.relative_humidity = -1;
        pub_DHT22Temp.publish(&DHT22_Temperature_msgs);
        pub_DHT22Humid.publish(&DHT22_Humidity_msgs);
        break;
      case DHT_BUS_HUNG:
        Serial.println("BUS Hung");
        DHT22_Temperature_msgs.temperature = 0;
        DHT22_Humidity_msgs.relative_humidity = 0;
        pub_DHT22Temp.publish(&DHT22_Temperature_msgs);
        pub_DHT22Humid.publish(&DHT22_Humidity_msgs);
        break;
      case DHT_ERROR_DATA_TIMEOUT:
        Serial.println("Data timeout");
        break;
      case DHT_ERROR_TOOQUICK:
        Serial.println("Polled to quick ");
        break;
      case DHT_ERROR_NOT_PRESENT:
        Serial.println("Nothing");
        break;
    }

    /* motion detection */
    PIR_msgs.data = digitalRead(PIR_PIN);
    pub_PIRstate.publish(&PIR_msgs); 
  
    if (PIR_msgs.data == true)
    {
      Serial.println("Somebody is in this area!");
    }
    else
    {
      Serial.println("No one!");
    }
    
    /* flame detection */
    Flame_msgs.data = analogRead(Flame_PIN);
    pub_Flame.publish(&Flame_msgs);
    
    /*Smoke detection MQ2 */
    MQ2_msgs.data = analogRead(MQ2_PIN);
    MQ2_msgs.data = MQ2_msgs.data / 1024 * 1000 + 200; // follow the recommendation regarding LPS on datasheet
    pub_MQ2Smoke.publish(&MQ2_msgs);
    
    /* Smoke detection MQ9 */
    MQ9_msgs_AI.data = analogRead(MQ9_PIN_AI);
    //MQ9_msgs_DI.data = digitalRead(MQ9_PIN_DI);
    MQ9_msgs_AI.data = MQ9_msgs_AI.data / 1024 * 1000 + 500; // follow the recommendation regarding LPS on datasheet
    pub_MQ9Smoke_AI.publish(&MQ9_msgs_AI);
    //pub_MQ9Smoke_DI.publish(&MQ9_msgs_DI);    
    
    /* Dust detection */ 
    digitalWrite(Dust_PIN_DO,LOW); // power on the LED
    delayMicroseconds(samplingTime);

    voMeasured = analogRead(Dust_PIN_AI);
    //Serial.println(voMeasured);
    delayMicroseconds(deltaTime);
    digitalWrite(Dust_PIN_DO,HIGH); // turn the LED off
    delayMicroseconds(sleepTime);
    
    calcVoltage = voMeasured * (5.0 / 1024.0); //restore volatage value
    Dust_msgs_VoMeasured.data = calcVoltage;
    pub_Dust_V.publish(&Dust_msgs_VoMeasured);
    
    //Dust_msgs.data = 0.17 * calcVoltage - 0.1; //linear eqaution taken from http://www.howmuchsnow.com/arduino/airquality/ ,Chris Nafis (c) 2012
    Dust_msgs.data = 0.2 * calcVoltage - 0.18; // this equation is appoximately calculated by using typical value shown in its datasheet 
    //Serial.println(calcVoltage);
    pub_Dust.publish(&Dust_msgs);

    
    /* timer */
    publisher_timer = millis() + 2000;
  }
  else;
  Security.spinOnce(); 
     
}
