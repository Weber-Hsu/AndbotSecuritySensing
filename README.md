# Andbot_SecuritySensing

* sensors:
	Smoke sensor -- MTARDSMOKE (MQ2) * 1
                            -- MQ9 * 1
	Temperature & humidity sensor -- DHT -22 * 1
	Motion sensor PIR * 1
	Flame sensor * 1
	Dust sensor * 1 

* controller:
	MEGA 2560 board * 1
	[arduino/Mega2560](https://www.arduino.cc/en/Main/arduinoBoardMega2560)

===============================================================

## Motion sensor SEN0171

	* Output format: Digital
	* ROS 
		* topic: /MotionDetection
		* msg type: Boolean (Lib: std_msgs/Bool)
	
**Pay attention: Once the IR signal disappears, the output pin will output low level delay roughly 2.3~3 seconds. So we can quickly establish a body motion detection application according to this feature.**  

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

## Temperature sensor AM2302 DHT22

* Output format: Digital
	* ROS 
		* topic: /CurTemperature
		* topic: /CurHumidity
		* msg type: double (Lib: sensor_msgs/Temperature & sensor_msgs/RelativeHumidity)
		
* Specifications
	* output: alibrated digital signal
	* power: 3.3~5.5V
	* Sensing range: humidity 0-100%RH; temperature -40~80Celsius
	* resolution: humidity 0.1%RH;temperature 0.1Celsius
	* accuracy: humidity+-2%RH (Max+-5%RH);temperature+-0.5Celsius

* reference
	* [Datasheet](https://cdn-shop.adafruit.com/datasheets/Digital+humidity+and+temperature+sensor+AM2302.pdf)
	* [dfrobot/wiki](http://www.dfrobot.com/wiki/index.php/DHT22_Temperature_and_humidity_module_SKU:SEN0137#More)

-------------------------------------------------------------------------------------------------

## Flame sensor

* Supply Voltage: 3.3V to 5V
* Interface: Analog
* reference 
	* http://www.dfrobot.com/index.php?route=product/product&product_id=195#.V0KYAHV97aV
	* http://www.dfrobot.com/wiki/index.php/Flame_sensor_SKU:_DFR0076

--------------------------------------------------------------------------------------------------

## Smoke MQ2

** Resistance value of MQ-2 is difference to various kinds and various concentration gases. So,When using this components, sensitivity adjustment is very necessary.

When accurately measuring, the proper alarm point for the gas detector should be determined after
considering the temperature and humidity influence. **

* reference
	* http://www.dfrobot.com/wiki/index.php?title=Analog_Gas_Sensor_SKU:SEN0127
	* https://www.seeedstudio.com/depot/datasheet/MQ-2.pdf
	* http://vanceance.blogspot.tw/2013/04/gas-sensor-with-arduino.html
	* http://www.powenko.com/wordpress/?p=5688
	* https://www.pololu.com/file/0J309/MQ2.pdf

--------------------------------------------------------------------------------------------------

## Smoke MQ9

* refernce 
	* http://domoticx.com/arduino-modules-mq-9-gas-sensor-methaan-lpg-koolstofmonoxide/
	* http://www.dfrobot.com/wiki/index.php/Analog_Gas_Sensor(MQ9)_SKU:SEN0134
	* http://www.dfrobot.com/image/data/SEN0134/SEN0134_MQ-9.pdf

---------------------------------------------------------------------------------------------------

## Sharp dust sensor

* Supply voltage: 5-7V

* reference
	* http://lafudo.blogspot.tw/2013/12/arduino-gp2y1010au0fpm25.html
	* http://www.dfrobot.com/index.php?route=product/product&filter_name=DUST%20SENSOR&product_id=867#.V0K2eXV97aV
	* http://www.dfrobot.com/wiki/index.php/Sharp_GP2Y1010AU
	* http://www.sharp-world.com/products/device/lineup/data/pdf/datasheet/gp2y1010au_appl_e.pdf

======================================================================

edited by shang wei (weber)



