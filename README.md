# AndbotSecuritySensing

This repository contains materials and instructions of practicing a partial function, security sensing, on Andbot. Details described below are divided into two section--hardware configurations and ROS implementation for upper level application. Other devices, Rugby, for example, can also use the following sensing methods. 

## Hardware configurations
### Sensor list

Item | Amount
-----|-------
MQ2 gas sensor | 1
MQ9 CO/Combustible Gas sensor | 1
DHT22 Temperature-Humidity sensor | 1
Flame sensor | 1
Dust sensor | 1
Motion sensor PIR | 1

**Note: Order of sensor has been defined. Please follow this in any future discussion.**

**Please note: Sensor type of the Dust sensing are still on the discussion.**

- [ ] **2016/06/08--I have acquired two different type of Dust sensing earlier today, and they are on the testing schedule.**

==========================================================
 
### Installation on andbot
* metal1 --> head [MEGA 2560 board](https://www.arduino.cc/en/Main/arduinoBoardMega2560)
* Pin configuration
		
**Please be carefull while setting up hardwares, and check twice before turning power on.**

Input/Output |  Pins
-------------|-------------
MQ2_PIN      |  A1 (Analog)
MQ9_PIN      |  A2 (Analog)
DHT22_PIN    |  22 (Digital) 
Flame_PIN    |  A0 (Analog)   
Dust_PIN_AI  |  A3 (Analog)   
Dust_PIN_DO  |  25 (Digital)
PIR_PIN      |  23 (Digital)

- [x] this is a complete item

==========================================================

### Upload Code 
1. Please copy all of files in the library folder to Arduino/library. (This is critical because it cannot be complie without those libraries.)
2. Main code: Please refer to folder AndbotSensingSecurity_ROS_Ver_2. 
	
**Folder " " and " " are my testing project. Please be aware! *Do not* use them.**

----------------------------------------------------------

## Sensor information published by using ROS 

**Topic of sensors will be published in every 2 seconds due to some of the sensor requirements. Moreover, there will be 2 more seconds at the beginning for sensor warm up.**

### Sensor Active List
* ROS
	* Topic: /SensorActiveList
 	*	Msg type: Int8MultiArray (Lib: std_msgs::Int8MultiArray)
	* Sensor ID definition: 

Sensor |  ID
-------|-------
 MQ2   |   0   
 MQ9   |   1   
 DHT22 |   2   
 Flame |   3   
 PM2.5 |   4   
 PIR   |   5
 
### MQ2 gas sensor
* Output format: Analog (intensity)
* ROS 
	* Topic: /MQ2LPG; /MQ2CO; MQ2SMOKE
	* Msg type: float (Lib: std_msgs::Float32)
	* Output: ppm 
		(**Approximation** is derived from datasheet; details are the following.) 

### MQ9 CO/Combustible Gas sensor
* Output format: Analog (intensity) 
	* Adjustable resistance RL = 5.4K ohm
* ROS 
	* Topic: /MQ9LPG; /MQ9CO; /MQ9CH4;
	* Msg type: float (Lib: std_msgs::Float32)
	* Output: ppm (Approximation is derived from datasheet; details are the following.)
		

### DHT22 Temperature-Humidity sensor
* Output format: Digital
* ROS 
	* Topic: /CurTemperature
	* Topic: /CurHumidity
	* Msg type: double (Lib: sensor_msgs/Temperature & sensor_msgs/RelativeHumidity)

### Flame sensor
* Output format: Analog (intensity)
* ROS 
	* Topic: /FlameDetection
	* Msg type: float (Lib: std_msgs::Float32)
	* Value: 0 ~ 1023

### Dust sensor -- pending. Need further testing.
* Output format: analog 
	* Physical meaning: signal can be transfered to Dust Density
* ROS 
	* Topic: /DustDetection
	* Msg type: float (Lib: std_msgs::Float32)
	* Unit: mg/m3
	
**Be aware of the wire color and pinout when setting up.**

### Motion sensor PIR
* Output format: Digital
* ROS 
	* Topic: /MotionDetection
	* Msg type: Boolean (Lib: std_msgs/Bool)
	
**Pay attention: Once the IR signal disappears, the output pin will output low level delay roughly 2.3~3 seconds.**  

----------------------------------------------------------
		
### Specifications and other useful reference of each sensor
####MQ2	
* Supply Voltage: 5V
* Adjustable resistance RL = 10K ohm
* This sensor is suitable for detecting LPG, i-butane, propane, methane ,alcohol, Hydrogen, smoke.
* **Preheat time: 24hr**
* **This sensor is pretty sensitive to temperature and humidity.**
* Sensor calibration procedure (written in the code already):
	1. Before running Calibration: 
		**It must be placed in anywhere with clean air.**
		Tune RL to 5k ohm, which is adjustable resistance on the sensor.
	2. Calibrating sensor resistance Ro in clean air:
	 	Pre-defined factor: Ro Clean Air factor 
		(Rs in clean air under given temperature and humidity is a constant，which is the “initial” resistance of the sensor named Ro.)
			Ro = Rs (sensor reading average in 500 samples) / Rfactor (derived from the datasheet)
* Measuring gas
	Pre-defined factor: LPGCurve; COCurve; SMOKECurve. (logy = a * logx + b)
	gas (ppm) = 10 ^ ( a * logx + b)
* Reference
	1. [dfrobot/wiki](http://www.dfrobot.com/wiki/index.php?title=Analog_Gas_Sensor_SKU:SEN0127)
	2. [datasheet](https://www.seeedstudio.com/depot/datasheet/MQ-2.pdf)
	3. [datasheet](https://www.pololu.com/file/0J309/MQ2.pdf)
	4. [example](http://vanceance.blogspot.tw/2013/04/gas-sensor-with-arduino.html)
	5. [example](http://www.powenko.com/wordpress/?p=5688)
	6. [common sense](http://www.tfci.org.tw/Fc/fc1-6.asp)			

####MQ9
* Supply Voltage: 5V
* Concentration:
	* 10-1000ppm CO
	* 100-10000ppm combustible gas
* Good sensitivity to CO/Combustible gas
* High sensitivity to Methane, Propane and CO
* **Preheat time: 48hr**
* **This sensor is pretty sensitive to temperature and humidity.**
**We desided not to use onboard digital output due to its unknown programmed threshold for alarm.**
* Sensor calibration procedure (written in the code already):
	1. Before running Calibration: 
		It must be placed in anywhere with clean air.
		Tune RL to 5k ohm, which is adjustable resistance on the sensor.
	2. Calibrating sensor resistance Ro in clean air:
    	Pre-defined factor: Ro Clean Air factor 
		Rs in clean air under given temperature and humidity is a constant，which is the “initial” resistance of the sensor named Ro.
		Ro = Rs (sensor reading average in 500 samples) / Rfactor (derived from the datasheet)
* Measuring gas
	Pre-defined factor: LPGCurve; COCurve; SMOKECurve. (logy = a * logx + b)
	gas (ppm) = 10 ^ ( a * logx + b)
* Reference 
	1. [datasheet](https://solarbotics.com/download.php?file=2274)
	2. [datasheet](http://www.dfrobot.com/image/data/SEN0134/SEN0134_MQ-9.pdf)
	3. [dfrobot/wiki](http://www.dfrobot.com/wiki/index.php/Analog_Gas_Sensor(MQ9)_SKU:SEN0134)
	4. [example](http://www.powenko.com/wordpress/?p=5688)
	5. [common sense](http://www.tfci.org.tw/Fc/fc1-6.asp)
	6. [CO safty level](http://www.nfa.gov.tw/main/Unit.aspx?ID=&MenuID=500&ListID=316)	

####DHT22
* Output: alibrated digital signal
* Power: 3.3~5.5V
* Sensing range: 
	* Humidity: 0-100% RH
	* Temperature: -40 ~ 80 Celsius
* Resolution: 
	* Humidity: 0.1% RH;
	* Temperature: 0.1 Celsius
* Accuracy: 
	* Humidity: +-2% RH (Max+-5%RH)
	* Temperature: +-0.5 Celsius
* Reference
	1. [Datasheet](https://cdn-shop.adafruit.com/datasheets/Digital+humidity+and+temperature+sensor+AM2302.pdf)
	2. [dfrobot/wiki](http://www.dfrobot.com/wiki/index.php/DHT22_Temperature_and_humidity_module_SKU:SEN0137#More)	

####FLAME	
* Supply Voltage: 3.3V to 5V
* Detection range: 20cm (4.8V) ~ 100cm (1V)
* Rang of Spectral Bandwidth : 760nm to 1100nm
* Responsive time : 15us
* Interface: Analog
* **The flame sensor's operating temperature is -25 degrees Celsius to 85 degrees Celsius, in the course of the flame it should be noted that the probe distance from the flame should not be too close inorder to avoid damage.**
* Reference 
	1. [dfrobot/product](http://www.dfrobot.com/index.php?route=product/product&product_id=195#.V0KYAHV97aV)
	2. [dfrobot/wiki](http://www.dfrobot.com/wiki/index.php/Flame_sensor_SKU:_DFR0076)
	3. [schemetics](http://www.dfrobot.com/image/data/DFR0076/V2.0/Flame%20Sensor%20SCH.pdf)
	4. [datasheet](https://github.com/Arduinolibrary/Source/blob/master/YG1006ataSheet.pdf?raw=true)

####PM2.5
* Supply voltage: 5-7V
* Operating temperature: -10-65 Celsius 
* Output range:
* Reference
	1. [datasheet](http://www.dfrobot.com/image/data/SEN0144/gp2y1010au_e.pdf)
	2. [datasheet](http://www.sharp-world.com/products/device/lineup/data/pdf/datasheet/gp2y1010au_appl_e.pdf)
	3. [example](http://lafudo.blogspot.tw/2013/12/arduino-gp2y1010au0fpm25.html)
	4. [dfrobot/](http://www.dfrobot.com/index.php?route=product/product&filter_name=DUST%20SENSOR&product_id=867#.V0K2eXV97aV)
	5. [dfrobot/wiki](http://www.dfrobot.com/wiki/index.php/Sharp_GP2Y1010AU)

####PIR		
* Input type: pyroelectic infrared. detecting infrared signals from moving person or animals.
* Output: switching signals
  High 3V
  Low 0V
* Power input: 3.3~5 V (It should not be bigger than 6V)
* Applications: body movements
* Working current: 15uA
* Detection distance: 7 m
* Reference 
		* [dfrobot/wiki](http://www.dfrobot.com/wiki/index.php/PIR_Motion_Sensor_V1.0_SKU:SEN0171)		


