#ifndef MOD_PM2dot5_h
#define MOD_PM2dot5_h

#if ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#endif

class MOD_PM2dot5 {
public:
		MOD_PM2dot5(int pin_AI, int pin_DO);
		float* read(bool print);

		float MOD_PM2dot5Calibration();
		float MOD_PM2dot5GetConcentration(float Humidity);
		float MOD_PM2dot5Read();
private:
		const int CALIBRATION_SAMPLES = 1000;
		const int READ_SAMPLES = 30;

		/* sampling timing of output pulse in Dust sensor (from Datasheet)*/
		const int samplingTime = 280; // LED Pulse Width = samplingTime + deltaTime = 320us
		const int deltaTime = 40;
		const int sleepTime = 9680; // period (per pulse) = 10ms, i.e, sleepingTime = 10ms - 320us = 9680 us

		int DN7C3CA006_pin_AI;
		int DN7C3CA006_pin_DO;

		float Vs = 0; // (unit: V)
		float Vo= 0; // (unit: V)
		float alpha = 0.6 ; //datasheet
		float beta = 0 ; //

		float MOD_PM2dot5VolCal(int raw_adc);
};
