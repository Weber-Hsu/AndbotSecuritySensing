#include <ros.h>

#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/RelativeHumidity.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
#include <stdio.h>
#include <Metro.h>

/* Sensor libraries */
#include <DHT22.h>      // temperauture sensor
#include <MQ2.h> 
#include <MQ9.h>
#include <MOD_PM2dot5.h>

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
/* ID    |  Sensor
 * :----:| :-------:
 *  0    |   MQ2
 *  1    |   DHT22
 *  2    |   Flame
 *  3    |   PM2.5
 */
DHT22 angelDHT22(DHT22_PIN);
MQ2 angelMQ2(MQ2_PIN);
MQ9 angelMQ9(MQ9_PIN_AI);
MOD_PM2dot5 angelMOD_PM2dot5(Dust_PIN_AI,Dust_PIN_DO);
unsigned int SensorID[5] = {0,1,2,3}; // 0: MQ2 ; 1: MQ9; 2: DHT22; 3: Flame; 4: PM2.5
bool SensorActiveStatus[5] = {true,true,true,true};
unsigned int SensorTotalNums = sizeof(SensorID)/2;

long publisher_timer ;
long publishPeriod= 2000; 

std_msgs::UInt8MultiArray SensorActiveList_msgs = {}; // List of sensor configured on andbot
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
ros::Publisher pub_SensorList("/SensorActiveList", & SensorActiveList_msgs);
ros::Publisher pub_DHT22Temp("/CurTemperature", &DHT22_Temperature_msgs);
ros::Publisher pub_DHT22Humid("/CurHumidity", &DHT22_Humidity_msgs);
ros::Publisher pub_PIRstate("/MotionDetection", &PIR_msgs);
ros::Publisher pub_Flame("/FlameDetection", &Flame_msgs);
//ros::Publisher pub_MQ2LPG("/MQ2LPG", & MQ2_msgs_LPG);
ros::Publisher pub_MQ2CO("/MQ2CO", & MQ2_msgs_CO);
//ros::Publisher pub_MQ2SMOKE("/MQ2SMOKE", & MQ2_msgs_SMOKE);
//ros::Publisher pub_MQ9LPG("/MQ9LPG", & MQ9_msgs_LPG);
//ros::Publisher pub_MQ9CO("/MQ9CO", & MQ9_msgs_CO);
//ros::Publisher pub_MQ9CH4("/MQ9CH4", & MQ9_msgs_CH4);
ros::Publisher pub_Dust("/DustDetection", & Dust_msgs);
ros::Publisher pub_Dust_V("/DustDetectionV", & Dust_msgs_VoMeasured);


/* temporary varibles for Dust sensing*/
float voMeasured = 0;
float calcVoltage = 0;

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
  Security.advertise(pub_MQ2CO);
  //Security.advertise(pub_MQ9Smoke_AI);
  //Security.advertise(pub_MQ9Smoke_DI);
  Security.advertise(pub_Dust);
  Security.advertise(pub_Dust_V);


  Serial.begin(1000000);
    // calculation for Numbers of Active Sensor
  SensorActiveList_msgs.data = (uint8_t*)malloc(sizeof(uint8_t)* SensorTotalNums );
  Security.advertise(pub_SensorList);

  unsigned int SensorActiveNums = 0 ;
  for (int i = 0 ; i < SensorTotalNums; i++)
  {
  	if (SensorActiveStatus[i] == true)
  	{
  		SensorActiveList_msgs.data[SensorActiveNums] = SensorID[i];
  		SensorActiveNums += 1;
  	}
  	else;
  }
  SensorActiveList_msgs.data_length = SensorActiveNums;

    /* sensor calibration */
  angelMQ2.MQCalibration();
  //angelMQ9.MQCalibration();
  angelMOD_PM2dot5.MOD_PM2dot5Calibration();

    //Warming up ...
  Serial.println("Please wait for warmup ...");
  while(warmup.check() == false);
  SensorReadyFlag = true;
  Serial.println("warmup finish");

}

void loop() {
  // put your main code here, to run repeatedly:

	// publish SensorActiveList repeatedly
	pub_SensorList.publish(&SensorActiveList_msgs);

  /* DHT22 reading... */
  DHT22_ERROR_t errorCode;

  if (publishPeriod.check() == true && SensorReadyFlag == true)
  {
    errorCode = angelDHT22.readData();
    switch (errorCode)
    {
      case DHT_ERROR_NONE:
        DHT22_Temperature_msgs.temperature = (double)angelDHT22.getTemperatureC();
        DHT22_Humidity_msgs.relative_humidity = (double)angelDHT22.getHumidity();
        pub_DHT22Temp.publish(&DHT22_Temperature_msgs);
        pub_DHT22Humid.publish(&DHT22_Humidity_msgs);
        break;
      case DHT_ERROR_CHECKSUM:
        Serial.println("sum error");
        break;
      case DHT_BUS_HUNG:
        Serial.println("BUS Hung");
        break;
      case DHT_ERROR_ACK_TOO_LONG:
		Serial.println("ACK time out ");
		break;
	  case DHT_ERROR_SYNC_TIMEOUT:
		Serial.println("Sync Timeout ");
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
    MQ2_msgs_CO.data = angelMQ2.readCO();
    //MQ2_msgs.data = MQ2_msgs.data / 1024 * 1000 + 200; // follow the recommendation regarding LPS on datasheet
    pub_MQ2CO.publish(&MQ2_msgs_CO);

    /* Smoke detection MQ9 */
//    MQ9_msgs_AI.data = analogRead(MQ9_PIN_AI);
//    //MQ9_msgs_DI.data = digitalRead(MQ9_PIN_DI);
//    //MQ9_msgs_AI.data = MQ9_msgs_AI.data / 1024 * 1000 + 500; // follow the recommendation regarding LPS on datasheet
//    pub_MQ9Smoke_AI.publish(&MQ9_msgs_AI);
    //pub_MQ9Smoke_DI.publish(&MQ9_msgs_DI);

    /* Dust detection */
    /* Dust detection */
    //		Dust_msgs_VoMeasured.data = angelMOD_PM2dot5.MOD_PM2dot5Read();
    //	  //	    Serial.print("Vo: ");
    //	  //	    Serial.println(Dust_msgs_VoMeasured.data);
    //		pub_Dust_V.publish(&Dust_msgs_VoMeasured);
    //
    //		Dust_msgs.data = angelMOD_PM2dot5.MOD_PM2dot5GetConcentration(DHT22_Humidity_msgs.data);
    //		pub_Dust.publish(&Dust_msgs);
  }
  else;
  Security.spinOnce();

}
