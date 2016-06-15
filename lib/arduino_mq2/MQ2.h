#ifndef MQ2_h
#define MQ2_h

#if ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#endif

class MQ2 {
public:
		MQ2(int pin);
		float* read(bool print);
		float readLPG();
		float readCO();
		float readSMOKE();

		float MQCalibration();
		int MQGetGasPercentage(float rs_ro_ratio, int gas_id);
private:
		const int CALIBRATION_SAMPLES =  50; // number of samples
		const int CALIBRATION_SAMPLE_INTERVAL = 500 ;// (ms)
		const int READ_SAMPLES  = 5; // number of samples
		const int READ_SAMPLE_INTERVAL  = 50; //(ms)
		const int GAS_LPG  = 0;
		const int GAS_CO  = 1;
		const int GAS_SMOKE  = 2;

		int mq_pin;
		int RL_VALUE = 5.22; // (k ohm)
		int RO_CLEAN_AIR_FACTOR = 9.83; // from datasheet

		float LPGCurve[3] = {2.3,0.21,-0.47};
		float COCurve[3] = {2.3,0.72,-0.34};
		float SmokeCurve[3] = {2.3,0.53,-0.44};
		float Ro = 10; // (k ohm)

		float MQRead();

		int MQGetPercentage(float rs_ro_ratio, float *pcurve);
		float MQResistanceCalculation(int raw_adc);
};
