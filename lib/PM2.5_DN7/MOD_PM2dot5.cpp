#include "MOD_PM2dot5.h"

#include "Arduino.h"

MOD_PM2dot5::MOD_PM2dot5(int pin_AI, int pin_DO){
	DN7C3CA006_pin_AI = pin_AI;
	DN7C3CA006_pin_DO = pin_DO;
	pinMode(DN7C3CA006_pin_DO, OUTPUT);
}

float MOD_PM2dot5::MOD_PM2dot5VolCal(int raw_adc)
{
	return (raw_adc * 5.0 / 1024.0);
}

float MOD_PM2dot5::MOD_PM2dot5Calibration()
{
	int i;
	float val = 0;

	for (i=0;i<CALIBRATION_SAMPLES;i++){
		digitalWrite(DN7C3CA006_pin_DO, LOW);  // power on the LED
		delayMicroseconds(samplingTime);
		val += MOD_PM2dot5VolCal(analogRead(DN7C3CA006_pin_AI));
		delayMicroseconds(deltaTime);
		digitalWrite(DN7C3CA006_pin_DO, HIGH); // turn the LED off
		delayMicroseconds(sleepTime);
	}
	Vs = val / CALIBRATION_SAMPLES;

	return Vs;
}

float MOD_PM2dot5::MOD_PM2dot5Read()
{
	int i;

	for (i=0;i<READ_SAMPLES;i++) {
		digitalWrite(DN7C3CA006_pin_DO, LOW);  // power on the LED
		delayMicroseconds(samplingTime);
		Vo += MOD_PM2dot5VolCal(analogRead(DN7C3CA006_pin_AI));
		delayMicroseconds(deltaTime);
		digitalWrite(DN7C3CA006_pin_DO, HIGH); // turn the LED off
		delayMicroseconds(sleepTime);
	}

	Vo = Vo / READ_SAMPLES;

	return Vo;
}

float MOD_PM2dot5::MOD_PM2dot5GetConcentration(float Humidity)
{
	float PM2dot5 = 0;
	Humidity = 60;

	if (Humidity < 50)
	{
		beta = 1;
	}
	else
	{
		beta = 1-0.01467 * (Humidity - 50); //datasheet
	}

	if (Vo > Vs)
	{
		return (PM2dot5 = alpha * beta * (Vo - Vs) * 1000.0); // (unit: ug/m3)
	}
	else
	{
		return 0;
	}
}
