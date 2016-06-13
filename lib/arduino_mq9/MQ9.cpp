#include "Arduino.h"
#include "MQ9.h"

MQ9::MQ9(int pin){
	mq_pin = pin;
}

float MQ9::readLPG(){
	float lpg = MQGetGasPercentage(MQRead()/Ro,GAS_LPG);
	return lpg;
}

float MQ9::readCO(){
	float CO = MQGetGasPercentage(MQRead()/Ro,GAS_CO);
	return CO;
}

float MQ9::readCH4(){
	float CH4 = MQGetGasPercentage(MQRead()/Ro,GAS_CH4);
	return CH4;
}

float MQ9::MQResistanceCalculation(int raw_adc)
{
	return (((float)RL_VALUE * (1023 - raw_adc) / raw_adc));
}

float MQ9::MQCalibration()
{
	int i;
	float val = 0;

	for (i=0;i<CALIBRATION_SAMPLES;i++){
		val += MQResistanceCalculation(analogRead(mq_pin));
	}
	val = val / CALIBRATION_SAMPLES;

	Ro = val / RO_CLEAN_AIR_FACTOR;

	return Ro;
}

float MQ9::MQRead()
{
	int i;
	float rs = 0;

	for (i=0;i<READ_SAMPLE;i++) {
		rs += MQResistanceCalculation(analogRead(mq_pin));
		delay(READ_SAMPLE_INTERVAL);
	}

	rs = rs / READ_SAMPLE;

	return rs;
}


int MQ9::MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
	if (gas_id == GAS_LPG){
		return MQGetPercentage(rs_ro_ratio, LPGCurve);
	} else if (gas_id == GAS_CO) {
		return MQGetPercentage(rs_ro_ratio, COCurve);
	} else if (gas_id == GAS_CH4) {
		return MQGetPercentage(rs_ro_ratio, CH4Curve);
	}
		return 0;
}

int MQ9::MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
	return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}
