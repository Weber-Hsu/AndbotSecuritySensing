#include "Arduino.h"
#include "MQ2.h"

MQ2::MQ2(int pin){
	mq_pin = pin;
}

float MQ2::readLPG(){
	float lpg = MQGetGasPercentage(MQRead()/Ro,GAS_LPG);

	return lpg;
}

float MQ2::readCO(){
	float CO = MQGetGasPercentage(MQRead()/Ro,GAS_CO);
	return CO;
}

float MQ2::readSMOKE(){
	float SMOKE = MQGetGasPercentage(MQRead()/Ro,GAS_SMOKE);
	return SMOKE;
}

float MQ2::MQResistanceCalculation(int raw_adc)
{
	return (((float)RL_VALUE * (1023 - raw_adc) / raw_adc));
}

float MQ2::MQCalibration()
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

float MQ2::MQRead()
{
	int i;
	float rs = 0;

	for (i=0;i<READ_SAMPLES;i++) {
		rs += MQResistanceCalculation(analogRead(mq_pin));
		delay(READ_SAMPLE_INTERVAL);
	}

	rs = rs / READ_SAMPLES;

	return rs;
}


int MQ2::MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
	if (gas_id == GAS_LPG){
		return MQGetPercentage(rs_ro_ratio, LPGCurve);
	} else if (gas_id == GAS_CO) {
		return MQGetPercentage(rs_ro_ratio, COCurve);
	} else if (gas_id == GAS_SMOKE) {
		return MQGetPercentage(rs_ro_ratio, SmokeCurve);
	}
		return 0;
}

int MQ2::MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
	return (pow(10,(((log(rs_ro_ratio) - pcurve[1])/pcurve[2]) + pcurve[0])));
}
