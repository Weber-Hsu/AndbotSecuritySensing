#ifndef MQ9_h
#define MQ9_h

#if ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#endif

class MQ9 {
public:
		MQ9(int pin);
		float* read(bool print);
		float readLPG();
		float readCO();
		float readCH4();

		float MQCalibration();
		int MQGetGasPercentage(float rs_ro_ratio, int gas_id);
		//void begin();
private:
		#define CALIBRATION_SAMPLE_TIMES 50
		#define CALIBRATION_SAMPLE_INTERVAL 500 // (ms)
		#define READ_SAMPLE_TIMES 5
		#define READ_SAMPLE_INTERVAL 50
		#define GAS_LPG 0
		#define GAS_CO 1
		#define GAS_CH4 2

		int mq_pin;
		int RL_VALUE = 1; // (k ohm)
		int RO_CLEAN_AIR_FACTOR = 9.9; // from datasheet

		float LPGCurve[3] = {2.3,0.322,-0.473};
		float COCurve[3] = {2.3,-0.097,-0.503};
		float CH4Curve[3] = {2.3,0.491,-0.38};
		float Ro = 10; // (k ohm)

		float MQRead();

		int MQGetPercentage(float rs_ro_ratio, float *pcurve);
		float MQResistanceCalculation(int raw_adc);
};
