#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/RelativeHumidity.h>
#include <std_msgs/Bool.h>
#include <stdio.h>
#include <Metro.h>

/* Sensor libraries */
#include <DHT22.h>      // temperauture sensor
#include <MQ2.h> 
#include <MQ9.h>

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
MQ2 andbotMQ2(MQ2_PIN);
MQ9 andbotMQ9(MQ9_PIN_AI);

long publisher_timer ;
Metro publishPeriod = Metro(2000); 

sensor_msgs::Temperature DHT22_Temperature_msgs; // DHT22 -temperture digital input
sensor_msgs::RelativeHumidity DHT22_Humidity_msgs; // DHT22 -Humidity digital input
std_msgs::Bool PIR_msgs; //PIR (motion sensor) digital input
std_msgs::Float32 Flame_msgs; // Flame sensor V2 analog input
std_msgs::Float32 MQ2_msgs_LPG; // MQ2 (smoke sensor) analog input
std_msgs::Float32 MQ2_msgs_CO;
std_msgs::Float32 MQ2_msgs_SMOKE;
std_msgs::Float32 MQ9_msgs_CO; // MQ9 (smoke sensor) analog input
std_msgs::Float32 MQ9_msgs_LPG;
std_msgs::Float32 MQ9_msgs_CH4;
//std_msgs::Bool MQ9_msgs_DI; // MQ9 (smoke sensor) digital input
std_msgs::Float32 Dust_msgs; // Sharp Optical Dust sensor analog input
std_msgs::Float32 Dust_msgs_VoMeasured; // Sharp Optical Dust sensor analog input

/*  define  ROS node and topics */
ros::NodeHandle Security;
ros::Publisher pub_DHT22Temp("/CurTemperature", &DHT22_Temperature_msgs);
ros::Publisher pub_DHT22Humid("/CurHumidity", &DHT22_Humidity_msgs);
ros::Publisher pub_PIRstate("/MotionDetection", &PIR_msgs);
ros::Publisher pub_Flame("/FlameDetection", &Flame_msgs);
ros::Publisher pub_MQ2LPG("/MQ2LPG", & MQ2_msgs_LPG);
ros::Publisher pub_MQ2CO("/MQ2CO", & MQ2_msgs_CO);
ros::Publisher pub_MQ2SMOKE("/MQ2SMOKE", & MQ2_msgs_SMOKE);
ros::Publisher pub_MQ9LPG("/MQ9LPG", & MQ9_msgs_LPG);
ros::Publisher pub_MQ9CO("/MQ9CO", & MQ9_msgs_CO);
ros::Publisher pub_MQ9CH4("/MQ9CH4", & MQ9_msgs_CH4);
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

Metro warmup = Metro(2000) ; //(msec)
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
  Security.advertise(pub_MQ2LPG);
  Security.advertise(pub_MQ2CO);
  Security.advertise(pub_MQ2SMOKE);
  Security.advertise(pub_MQ9LPG);
  Security.advertise(pub_MQ9CO);
  Security.advertise(pub_MQ9CH4);
  //Security.advertise(pub_MQ9Smoke_DI);
  Security.advertise(pub_Dust);
  Security.advertise(pub_Dust_V);

  Serial.begin(115200);

  /* sensor calibration */
  andbotMQ2.MQCalibration();
  andbotMQ9.MQCalibration();
}

void loop() {
  // put your main code here, to run repeatedly:

  //warmup sequence
  if (SensorReadyFlag == false)
  {
    if (warmup.check() == false)
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

  if (publishPeriod.check() == true && SensorReadyFlag == true)
  {
    errorCode = andbotDHT22.readData();
    switch (errorCode)
    {
      case DHT_ERROR_NONE:
        DHT22_Temperature_msgs.temperature = (double)andbotDHT22.getTemperatureC();
        DHT22_Humidity_msgs.relative_humidity = (double)andbotDHT22.getHumidity();
        pub_DHT22Temp.publish(&DHT22_Temperature_msgs);
        pub_DHT22Humid.publish(&DHT22_Humidity_msgs);
        break;
      case DHT_ERROR_CHECKSUM:
        Serial.println("sum error");
        break;
      case DHT_BUS_HUNG:
        Serial.println("BUS Hung");
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
    MQ2_msgs_LPG.data = andbotMQ2.readLPG();// follow the recommendation regarding LPS on datasheet
    MQ2_msgs_CO.data = andbotMQ2.readCO();
    MQ2_msgs_SMOKE.data = andbotMQ2.readSMOKE();
    pub_MQ2LPG.publish(&MQ2_msgs_LPG);
    pub_MQ2CO.publish(&MQ2_msgs_CO);
    pub_MQ2SMOKE.publish(&MQ2_msgs_SMOKE);

    /* Smoke detection MQ9 */
    MQ9_msgs_LPG.data = andbotMQ9.readLPG();
    MQ9_msgs_CO.data = andbotMQ9.readCO();
    MQ9_msgs_CH4.data = andbotMQ9.readCH4();
    Serial.print("CH4: ");
    Serial.println(MQ9_msgs_CH4.data);
    //MQ9_msgs_DI.data = digitalRead(MQ9_PIN_DI);
    pub_MQ9LPG.publish(&MQ9_msgs_LPG);
    pub_MQ9CO.publish(&MQ9_msgs_CO);
    pub_MQ9CH4.publish(&MQ9_msgs_CH4);
    //pub_MQ9Smoke_DI.publish(&MQ9_msgs_DI);

    /* Dust detection */
    digitalWrite(Dust_PIN_DO, LOW); // power on the LED
    delayMicroseconds(samplingTime);

    voMeasured = analogRead(Dust_PIN_AI);

    delayMicroseconds(deltaTime);
    digitalWrite(Dust_PIN_DO, HIGH); // turn the LED off
    delayMicroseconds(sleepTime);

    calcVoltage = voMeasured * (5.0 / 1024.0); //restore volatage value
    Dust_msgs_VoMeasured.data = calcVoltage;
    pub_Dust_V.publish(&Dust_msgs_VoMeasured);

    //Dust_msgs.data = 0.17 * calcVoltage - 0.1; //linear eqaution taken from http://www.howmuchsnow.com/arduino/airquality/ ,Chris Nafis (c) 2012
    Dust_msgs.data = 0.2 * calcVoltage - 0.18; // this equation is appoximately calculated by using typical value shown in its datasheet
    pub_Dust.publish(&Dust_msgs);

    /* timer */
    //publisher_timer = millis() + publishPeriod;
  }
  else;
  Security.spinOnce();

}
