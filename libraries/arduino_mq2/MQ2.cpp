#include "Arduino.h"
#include "MQ2.h"

MQ2::MQ2(int pin){
	mq_pin = pin;
}

//void MQ2::begin(){
//	Ro = MQCalibration();
//	Serial.pring("Ro: ");
//	Serial.print(Ro);
//	Serial.println(" kohm");
//}

float MQ2::readLPG(){
	float lpg = MQGetGasPercentage(MQRead()/Ro,GAS_LPG);
	Serial.print("LPG: ");
	Serial.print(lpg);
	Serial.println(" ppm");
	return lpg;
}

float MQ2::readCO(){
	float CO = MQGetGasPercentage(MQRead()/Ro,GAS_CO);
	Serial.print("CO: ");
	Serial.print(CO);
	Serial.println(" ppm");
	return CO;
}

float MQ2::readSMOKE(){
	float SMOKE = MQGetGasPercentage(MQRead()/Ro,GAS_SMOKE);
	Serial.print("SMOKE: ");
	Serial.print(SMOKE);
	Serial.println(" ppm");
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

	for (i=0;i<CALIBRATION_SAMPLE_TIMES;i++){
		val += MQResistanceCalculation(analogRead(mq_pin));
	}
	val = val / CALIBRATION_SAMPLE_TIMES;

	val = val / RO_CLEAN_AIR_FACTOR;

	Serial.print("Ro: ");
	Serial.print(val);
	Serial.println(" kohm");

	return val;
}

float MQ2::MQRead()
{
	int i;
	float rs = 0;

	for (i=0;i<READ_SAMPLE_TIMES;i++) {
		rs += MQResistanceCalculation(analogRead(mq_pin));
		delay(READ_SAMPLE_INTERVAL);
	}

	rs = rs / READ_SAMPLE_TIMES;

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
