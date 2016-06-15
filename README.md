# AndbotSecuritySensing

This repository contains materials and instructions of practicing a partial function, security sensing, on Andbot. Details described below are divided into two section--hardware configurations and ROS implementation for upper level application. Other devices, Rugby, for example, can also use the following sensing methods. 

* Table of contents
	* Hardware configurations
		* Sensor list
		* Installation on Andbot
		* Upload code
	* Sensor information published by using ROS 
		* Sensor Active List
		* Topics of each sensor
	* Specifications and other useful reference of each sensor

## Development History

- [x] 2016/06/13--MQ9 have been **removed** from this project because it has similar features compared to  MQ2. On the way of using MQ2 we select an alarm point 200ppm on CO Curve due to the fact that people usually cannot stand 200ppm of CO within 2~3hrs indoors according to safty level on [this] (http://www.nfa.gov.tw/main/Unit.aspx?ID=&MenuID=500&ListID=316) list. Also, the alarm point is before other detectable gases; that is, when readings from MQ2 increases, unknown but dangerous gases would be exist in certain space. We should be aware of the readings before sensor alarming.        
- [ ] **2016/06/08--I have acquired two different type of Dust sensing earlier today, and they are on the testing schedule.**

## Hardware configurations
### Sensor list

Item | Amount
-----|:------:
MQ2 gas sensor | 1
MQ9 CO/Combustible Gas sensor | 1
DHT22 Temperature-Humidity sensor | 1
Flame sensor | 1
Dust sensor | 1
Motion sensor PIR | 1

**Note: Order of sensor has been defined. Please follow this in any future discussion.**

**Please note: Sensor type of the Dust sensing are still on the discussion.**

==========================================================
 
### Installation on Andbot
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

**Please refer to the section--Specifications and other useful reference of each sensor if you encounters any problem.**

==========================================================

### Upload Code 

**All programs are included in this repository.**

1. **Please copy all of files in the "lib" folder to Arduino/library. (This is critical because it cannot be complie without those libraries.)**
2. **Main code: Please refer to folder MetalHeadVerROSSensingLED.** 
3. ROS Usage:
	1. Open a terminal and type "roscore".
	2. Open another terminal and type "rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=57600".
	Note: ttyACM0 refers to your serial port. It might be ttyACM1 or other.
	3. Open another terminal and type "rostopic list" to check ROS topics are ready.
	4. Type "rostopic echo /....." to listen to any topic. 
	
**Folder "Temporary Version" and "src" are debugging Version, and folder "MetalHeadPreviousVersion" contains program that will only turn the LED on. Please be aware! *Do not* use them.**



----------------------------------------------------------

## Sensor information published using ROS 

**Topic of sensors will be published in every 2 seconds due to some of the sensor requirements. Moreover, there will be 2 more seconds at the beginning for sensor warmup.**

### Sensor Active List
* ROS
	* Topic: /SensorActiveList
 	*	Msg type: UInt8MultiArray (Lib: std_msgs::UInt8MultiArray)
	* Sensor ID definition: 

Sensor |  ID
-------|:------:
 MQ2   |   0  
 DHT22 |   1   
 Flame |   2   
 PM2.5 |   3   
 PIR   |   4
 
### MQ2 gas sensor
* Output format: Analog (intensity)
* ROS 
	* Topic: /MQ2CO
	* Msg type: float (Lib: std_msgs::Float32)
	* Output: ppm 
		(**Approximation** is derived from datasheet; please refer to section--Specifications and other useful reference of each sensor for details.)
	* Alarm point: CO: 200 ppm 

### MQ9 CO/Combustible Gas sensor -- **Cancelled**
* Output format: Analog (intensity) 
* ROS 
	* Topic: /MQ9LPG; /MQ9CO; /MQ9CH4;
	* Msg type: float (Lib: std_msgs::Float32)
	* Output: ppm (**Approximation** is derived from datasheet; please refer to section--Specifications and other useful reference of each sensor for details.)
	* Pending reason: We believe that this breakout board were designed not properly. There is no adjustable resistance for analog output. 
		

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

### Dust sensor -- pending. 
* Output format: analog 
	* Physical meaning: signal can be transfered to Dust Density
* ROS 
	* Topic: /PM2dot5Detection
	* Msg type: float (Lib: std_msgs::Float32)
	* Unit: ug/m3

### Motion sensor PIR
* Output format: Digital
* ROS 
	* Topic: /MotionDetection
	* Msg type: Boolean (Lib: std_msgs/Bool)
	
**Pay attention: Once the IR signal disappears, the output pin will output low level delay roughly 2.3~3 seconds.**  

----------------------------------------------------------		
### Specifications and other useful reference of each sensor
#### MQ2	
* Supply Voltage: 5V
* Adjustable resistance RL = 10K ohm
* This sensor is suitable for detecting LPG, i-butane, propane, methane ,alcohol, Hydrogen, smoke.
* **Preheat time: 24hr**
* **This sensor is pretty sensitive to temperature and humidity.**
* Sensor calibration procedure (written in the code already):
	1. Before running Calibration:
		* It must be placed in anywhere with clean air.
		* Tune RL to 5k ohm, which is adjustable resistance on the sensor.
	2. Calibrating sensor resistance Ro in clean air:
		* Pre-defined factor: Ro Clean Air factor. 
			(Rs in clean air under given temperature and humidity is a constant，which is the “initial” resistance of the sensor named Ro.)
		*	Ro = Rs (sensor reading average in 500 samples) / Rfactor (derived from the datasheet)
* Measuring gas
		* Pre-defined factor: LPGCurve; COCurve; SMOKECurve. (logy = a * logx + b)
	  * Calculation: gas (ppm) = 10 ^ ( a * logx + b)
* Reference
	1. [dfrobot/wiki](http://www.dfrobot.com/wiki/index.php?title=Analog_Gas_Sensor_SKU:SEN0127)
	2. [datasheet](https://www.seeedstudio.com/depot/datasheet/MQ-2.pdf),[datasheet](https://www.pololu.com/file/0J309/MQ2.pdf)
	3. [example](http://vanceance.blogspot.tw/2013/04/gas-sensor-with-arduino.html),[example](http://www.powenko.com/wordpress/?p=5688)
	4. [common sense](http://www.tfci.org.tw/Fc/fc1-6.asp)			

#### MQ9 -- **Cancelled**
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
		* It must be placed in anywhere with clean air.
	2. Calibrating sensor resistance Ro in clean air:
		* Pre-defined factor: Ro Clean Air factor. Rs in clean air under given temperature and humidity is a constant，which is the “initial” resistance of the sensor named Ro.
		* Ro = Rs (sensor reading average in 500 samples) / Rfactor (derived from the datasheet)
* Measuring gas
	* Pre-defined factor: LPGCurve; COCurve; SMOKECurve. (logy = a * logx + b)
	* Calculation: gas (ppm) = 10 ^ ( a * logx + b)
* Reference 
	1. [datasheet](https://solarbotics.com/download.php?file=2274),[datasheet](http://www.dfrobot.com/image/data/SEN0134/SEN0134_MQ-9.pdf)
	2. [dfrobot/wiki](http://www.dfrobot.com/wiki/index.php/Analog_Gas_Sensor(MQ9)_SKU:SEN0134), [example](http://www.powenko.com/wordpress/?p=5688)
	3. [common sense](http://www.tfci.org.tw/Fc/fc1-6.asp), [CO safty level](http://www.nfa.gov.tw/main/Unit.aspx?ID=&MenuID=500&ListID=316)
	4. **[schematic](http://www.sunrom.com/p/combustible-gas-co-gas-sensor-mq9)**	

#### DHT22
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

#### FLAME	
* Supply Voltage: 3.3V to 5V
* Detection range: 20cm (4.8V) ~ 100cm (1V)
* Rang of Spectral Bandwidth : 760nm to 1100nm
* Responsive time : 15us
* Interface: Analog
* **The flame sensor's operating temperature is -25 degrees Celsius to 85 degrees Celsius, in the course of the flame it should be noted that the probe distance from the flame should not be too close inorder to avoid damage.**
* Reference 
	1. [dfrobot/product](http://www.dfrobot.com/index.php?route=product/product&product_id=195#.V0KYAHV97aV),[dfrobot/wiki](http://www.dfrobot.com/wiki/index.php/Flame_sensor_SKU:_DFR0076)
	2. [schemetics](http://www.dfrobot.com/image/data/DFR0076/V2.0/Flame%20Sensor%20SCH.pdf)
	3. [datasheet](https://github.com/Arduinolibrary/Source/blob/master/YG1006ataSheet.pdf?raw=true)

#### PM2.5
* Supply voltage: 5-7V
* Operating temperature: -10-65 Celsius 
* Output range:
* **Be aware of the wire color and pinout when setting up.**
* Reference
	1. [datasheet](http://www.dfrobot.com/image/data/SEN0144/gp2y1010au_e.pdf)
	2. [datasheet](http://www.sharp-world.com/products/device/lineup/data/pdf/datasheet/gp2y1010au_appl_e.pdf)
	3. [example](http://lafudo.blogspot.tw/2013/12/arduino-gp2y1010au0fpm25.html)
	4. [dfrobot/](http://www.dfrobot.com/index.php?route=product/product&filter_name=DUST%20SENSOR&product_id=867#.V0K2eXV97aV)
	5. [dfrobot/wiki](http://www.dfrobot.com/wiki/index.php/Sharp_GP2Y1010AU)
	6. [safty level](http://taqm.epa.gov.tw/taqm/tw/fpmi.htm)

#### PIR		
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


