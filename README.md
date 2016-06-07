# AndbotSecuritySensing
* Hardware configurations
	* Sensors
		* MQ2 gas sensor * 1
		* MQ9 CO/Combustible Gas sensor * 1
		* DHT22 Temperature-Humidity sensor * 1
		* Motion sensor PIR * 1
		* Flame sensor * 1
		* Dust sensor * 1 
	* Controller:
		* [MEGA 2560 board](https://www.arduino.cc/en/Main/arduinoBoardMega2560) * 1
	* Pin configuration
		 Input/Output |  Pins
		--------------|-------------
		 DHT22_PIN    |  22  
		 PIR_PIN      |  23   
		 Flame_PIN    |  A0    
		 MQ2_PIN      |  A1    
		 MQ9_PIN_AI   |  A2    
		 Dust_PIN_AI  |  A3    
		 Dust_PIN_DO  |  25    

# Implementation of each sensor on ROS

# Sensor information published by ROS
===============================================================

## Motion sensor PIR
* Output format: Digital
* ROS 
	* Topic: /MotionDetection
	* Msg type: Boolean (Lib: std_msgs/Bool)
	
	**Pay attention: Once the IR signal disappears, the output pin will output low level delay roughly 2.3~3 seconds.**  

* Specifications
  * Input type: pyroelectic infrared.
    detecting infrared signals from moving person or animals.
  * Output: switching signals
    High 3V
    Low 0V
  * Power input: 3.3~5 V (It should not be bigger than 6V)
  * Applications: body movements
  * Working current: 15uA
  * Detection distance: 7 m
* Reference 
  * [dfrobot/wiki](http://www.dfrobot.com/wiki/index.php/PIR_Motion_Sensor_V1.0_SKU:SEN0171)

-------------------------------------------------------------------------------------------------

## DHT22 Temperature-Humidity sensor
* Output format: Digital
* ROS 
	* Topic: /CurTemperature
	* Topic: /CurHumidity
	* Msg type: double (Lib: sensor_msgs/Temperature & sensor_msgs/RelativeHumidity)
		
* Specifications
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
	* [Datasheet](https://cdn-shop.adafruit.com/datasheets/Digital+humidity+and+temperature+sensor+AM2302.pdf)
	* [dfrobot/wiki](http://www.dfrobot.com/wiki/index.php/DHT22_Temperature_and_humidity_module_SKU:SEN0137#More)

-------------------------------------------------------------------------------------------------

## Flame sensor
* Output format: Analog (intensity)
* ROS 
	* Topic: /FlameDetection
	* Msg type: float (Lib: std_msgs::Float32)
	* Value: 0 ~ 1023
	
	**The flame sensor's operating temperature is -25 degrees Celsius to 85 degrees Celsius, in the course of the flame it should be noted that the probe distance from the flame should not be too close inorder to avoid damage.**
	
* Specifications
	* Supply Voltage: 3.3V to 5V
	* Detection range: 20cm (4.8V) ~ 100cm (1V)
	* Rang of Spectral Bandwidth : 760nm to 1100nm
	* Responsive time : 15us
	* Interface: Analog

* Reference 
	* [dfrobot/product](http://www.dfrobot.com/index.php?route=product/product&product_id=195#.V0KYAHV97aV)
	* [dfrobot/wiki](http://www.dfrobot.com/wiki/index.php/Flame_sensor_SKU:_DFR0076)
	* [schemetics](http://www.dfrobot.com/image/data/DFR0076/V2.0/Flame%20Sensor%20SCH.pdf)
	* [datasheet](https://github.com/Arduinolibrary/Source/blob/master/YG1006ataSheet.pdf?raw=true)

--------------------------------------------------------------------------------------------------

## MQ2 gas sensor
* Output format: Analog (intensity)
* ROS 
	* Topic: /MQ2
	* Msg type: float (Lib: std_msgs::Float32)
	* Adjustable resistance RL = 10K ohm

	**Preheat time: 24hr**

	**Resistance value of MQ-2 is difference to various kinds and various concentration gases. So,When using this components, sensitivity adjustment is very necessary.**
	**When accurately measuring, the proper alarm point for the gas detector should be determined after considering the temperature and humidity influence.**
	
	**There is a adjustable resistor that can be tune for suitable sensitivity.**
	
	*This sensor is pretty sensitive to temperature and humidity.*
	
* Specifications
	* Supply Voltage: 5V
	* This sensor is suitable for detecting LPG, i-butane, propane, methane ,alcohol, Hydrogen, smoke.

* Reference
	1. [dfrobot/wiki](http://www.dfrobot.com/wiki/index.php?title=Analog_Gas_Sensor_SKU:SEN0127)
	2. [datasheet](https://www.seeedstudio.com/depot/datasheet/MQ-2.pdf)
	3. [datasheet](https://www.pololu.com/file/0J309/MQ2.pdf)
	4. [example](http://vanceance.blogspot.tw/2013/04/gas-sensor-with-arduino.html)
	5. [example](http://www.powenko.com/wordpress/?p=5688)
	6. [common sense](http://www.tfci.org.tw/Fc/fc1-6.asp)

--------------------------------------------------------------------------------------------------

## MQ9 CO/Combustible Gas sensor
* Output format: Analog (intensity) & Digital (Alarm point?)
	* Adjustable resistance RL = 5.4K ohm
* ROS 
	* Topic: /MQ9
	* Msg type: float (Lib: std_msgs::Float32) 
	
	**Preheat time: 48hr**

	*We desided not to use onboard digital output due to its unknown programmed threshold for alarm.*
**This sensor is pretty sensitive to temperature and humidity.**

* Specifications
	* Supply Voltage: 5V
	* Concentration:
		* 10-1000ppm CO
		* 100-10000ppm combustible gas
		* Good sensitivity to CO/Combustible gas
		* High sensitivity to Methane, Propane and CO

* Reference 
	1. [datasheet](https://solarbotics.com/download.php?file=2274)
	2. [datasheet](http://www.dfrobot.com/image/data/SEN0134/SEN0134_MQ-9.pdf)
	3. [dfrobot/wiki](http://www.dfrobot.com/wiki/index.php/Analog_Gas_Sensor(MQ9)_SKU:SEN0134)
	4. [example](http://www.powenko.com/wordpress/?p=5688)
	5. [common sense](http://www.tfci.org.tw/Fc/fc1-6.asp)
	6. [CO safty level](http://www.nfa.gov.tw/main/Unit.aspx?ID=&MenuID=500&ListID=316)

---------------------------------------------------------------------------------------------------

## Dust sensor -- pending. Need further discussion.
* Output format: analog 
	* Physical meaning: signal can be transfered to Dust Density
* ROS 
	* Topic: /DustDetection
	* Msg type: float (Lib: std_msgs::Float32)
	* Unit: mg/m3
	
	**Be aware of the wire color and pinout when setting up.**
* Specification
	* Supply voltage: 5-7V
	* Operating temperature: -10-65 Celsius 
	* Output range:

* Reference
	1. [datasheet](http://www.dfrobot.com/image/data/SEN0144/gp2y1010au_e.pdf)
	2. [datasheet](http://www.sharp-world.com/products/device/lineup/data/pdf/datasheet/gp2y1010au_appl_e.pdf)
	3. [example](http://lafudo.blogspot.tw/2013/12/arduino-gp2y1010au0fpm25.html)
	4. [dfrobot/](http://www.dfrobot.com/index.php?route=product/product&filter_name=DUST%20SENSOR&product_id=867#.V0K2eXV97aV)
	5. [dfrobot/wiki](http://www.dfrobot.com/wiki/index.php/Sharp_GP2Y1010AU)

======================================================================

edited by shang wei (Weber)



