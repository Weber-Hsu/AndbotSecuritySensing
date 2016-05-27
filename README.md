# AndbotSecuritySensing
* sensors
	* MQ2 gas sensor * 1
	* MQ9 CO/Combustible Gas sensor * 1
	* DHT22 Temperature-Humidity sensor * 1
	* Motion sensor PIR * 1
	* Flame sensor * 1
	* Dust sensor * 1 

* controller:
	* MEGA 2560 board * 1
	[arduino/Mega2560](https://www.arduino.cc/en/Main/arduinoBoardMega2560)

===============================================================

## Motion sensor PIR
* Output format: Digital
* ROS 
	* topic: /MotionDetection
	* msg type: Boolean (Lib: std_msgs/Bool)
	
	**Pay attention: Once the IR signal disappears, the output pin will output low level delay roughly 2.3~3 seconds.**  

* Specifications
  * input type: pyroelectic infrared.
    detecting infrared signals from moving person or animals.
  * output: switching signals
    High 3V
    Low 0V
  * power input: 3.3~5 V (It should not be bigger than 6V)
  * applications: body movements
  * working current: 15uA
  * detection distance: 7 m
* reference 
  * [dfrobot/wiki](http://www.dfrobot.com/wiki/index.php/PIR_Motion_Sensor_V1.0_SKU:SEN0171)

-------------------------------------------------------------------------------------------------

## DHT22 Temperature-Humidity sensor
* Output format: Digital
* ROS 
	* topic: /CurTemperature
	* topic: /CurHumidity
	* msg type: double (Lib: sensor_msgs/Temperature & sensor_msgs/RelativeHumidity)
		
* Specifications
	* output: alibrated digital signal
	* power: 3.3~5.5V
	* Sensing range: 
		* humidity: 0-100% RH
		* temperature: -40 ~ 80 Celsius
	* resolution: 
		* humidity: 0.1% RH;
		* temperature: 0.1 Celsius
	* accuracy: 
		* humidity: +-2% RH (Max+-5%RH)
		* temperature: +-0.5 Celsius

* reference
	* [Datasheet](https://cdn-shop.adafruit.com/datasheets/Digital+humidity+and+temperature+sensor+AM2302.pdf)
	* [dfrobot/wiki](http://www.dfrobot.com/wiki/index.php/DHT22_Temperature_and_humidity_module_SKU:SEN0137#More)

-------------------------------------------------------------------------------------------------

## Flame sensor
* Output format: Analog (intensity)
* ROS 
	* topic: /FlameDetection
	* msg type: float (Lib: std_msgs::Float32)
	* value: 0 ~ 1023
	
	**The flame sensor's operating temperature is -25 degrees Celsius to 85 degrees Celsius, in the course of the flame it should be noted that the probe distance from the flame should not be too close inorder to avoid damage.**
	
* Specifications
	* Supply Voltage: 3.3V to 5V
	* Detection range: 20cm (4.8V) ~ 100cm (1V)
	* Rang of Spectral Bandwidth : 760nm to 1100nm
	* Responsive time : 15us
	* Interface: Analog

* reference 
	* [dfrobot/product](http://www.dfrobot.com/index.php?route=product/product&product_id=195#.V0KYAHV97aV)
	* [dfrobot/wiki](http://www.dfrobot.com/wiki/index.php/Flame_sensor_SKU:_DFR0076)
	* [schemetics](http://www.dfrobot.com/image/data/DFR0076/V2.0/Flame%20Sensor%20SCH.pdf)
	* [datasheet](https://github.com/Arduinolibrary/Source/blob/master/YG1006ataSheet.pdf?raw=true)

--------------------------------------------------------------------------------------------------

## MQ2 gas sensor
* Output format: Analog (intensity)
* ROS 
	* topic: /ConcentrationMQ2 -- temporary define.
	* To be determined: which type of gas should we reference? CO? LPG?
	* Recommendation from the datasheet -- please refer to reference 2 shown below. 
	* msg type: float (Lib: std_msgs::Float32)

	** Resistance value of MQ-2 is difference to various kinds and various concentration gases. So,When using this components, sensitivity adjustment is very necessary.
	When accurately measuring, the proper alarm point for the gas detector should be determined after
	considering the temperature and humidity influence. **
	
	**There is a adjustable resistor that can be tune for suitable sensitivity.**
	
	*This sensor is pretty sensitive to temperature and humidity.*
	
* Specifications
	* Supply Voltage: 5V
	* This sensor is suitable for detecting LPG, i-butane, propane, methane ,alcohol, Hydrogen, smoke.

* reference
	1. [dfrobot/wiki](http://www.dfrobot.com/wiki/index.php?title=Analog_Gas_Sensor_SKU:SEN0127)
	2. [datasheet](https://www.seeedstudio.com/depot/datasheet/MQ-2.pdf)
	3. [datasheet](https://www.pololu.com/file/0J309/MQ2.pdf)
	4. [example](http://vanceance.blogspot.tw/2013/04/gas-sensor-with-arduino.html)
	5. [example](http://www.powenko.com/wordpress/?p=5688)

--------------------------------------------------------------------------------------------------

## MQ9 CO/Combustible Gas sensor
* Output format: Analog (intensity) & Digital (Alarm point?)
	* To be determined: which type of gas should we reference? CO? LPG?
	* Recommendation from the datasheet -- please refer to reference 1 shown below. *
	* There is a adjustable resistor that can be tune for suitable sensitivity. * 
* ROS 
	* topic: /ConcentrationMQ9 -- temporary define.
	* msg type: float (Lib: std_msgs::Float32) 
	
*We desided not to use onboard digital output due to its unknown programmed threshold for alarm.*
**This sensor is pretty sensitive to temperature and humidity.**

* Specifications
	* Supply Voltage: 5V
	* Concentration:
		* 10-1000ppm CO
		* 100-10000ppm combustible gas
		* Good sensitivity to CO/Combustible gas
		* High sensitivity to Methane, Propane and CO

* reference 
	1. [datasheet](https://solarbotics.com/download.php?file=2274)
	2. [datasheet](http://www.dfrobot.com/image/data/SEN0134/SEN0134_MQ-9.pdf)
	3. [dfrobot/wiki](http://www.dfrobot.com/wiki/index.php/Analog_Gas_Sensor(MQ9)_SKU:SEN0134)
	4. [example](http://www.powenko.com/wordpress/?p=5688)

---------------------------------------------------------------------------------------------------

## Dust sensor
* Output format: analog 
	* physical meaning: signal can be transfered to Dust Density (needed)
* ROS 
	* topic: /DustDetection
	* msg type: float (Lib: std_msgs::Float32)
	
	**Be aware of the wire color and pinout when setting up.**
* Specification
	* Supply voltage: 5-7V
	* Operating temperature: -10-65 Celsius 
	* output range: please refer to reference 1 fig.3. It shows voltage against Dust density.

* reference
	1. [datasheet](http://www.dfrobot.com/image/data/SEN0144/gp2y1010au_e.pdf)
	2. [datasheet](http://www.sharp-world.com/products/device/lineup/data/pdf/datasheet/gp2y1010au_appl_e.pdf)
	3. [example](http://lafudo.blogspot.tw/2013/12/arduino-gp2y1010au0fpm25.html)
	4. [dfrobot/](http://www.dfrobot.com/index.php?route=product/product&filter_name=DUST%20SENSOR&product_id=867#.V0K2eXV97aV)
	5. [dfrobot/wiki](http://www.dfrobot.com/wiki/index.php/Sharp_GP2Y1010AU)

======================================================================

edited by shang wei (Weber)



