#ifndef _SERVO_ADC_READ_H
#define _SERVO_ADC_READ_H

//
//#define LB_TIBIA_CH 1
//#define LB_FEMUR_CH 0
//#define LB_SHOUL_CH 7
//
//#define LF_TIBIA_CH 11
//#define LF_FEMUR_CH 10
//#define LF_SHOUL_CH 8
//
//#define RF_TIBIA_CH 3
//#define RF_FEMUR_CH 2
//#define RF_SHOUL_CH 5
//
//#define RB_TIBIA_CH 4
//#define RB_FEMUR_CH 12
//#define RB_SHOUL_CH 6

#define BATT_CH 13

#define bufLen 14


typedef struct servoAngleReader {
	ADC_HandleTypeDef* servoADC;
	uint16_t ADC_buf[bufLen];
	double servoAngles[bufLen];
} servoAngleReader;



void servoReaderStart(servoAngleReader* reader) {
	HAL_ADC_Start_DMA(reader->servoADC, (uint32_t*)reader->ADC_buf, bufLen);
}



uint16_t getADCValue(servoAngleReader* reader, uint8_t servo_channel) {
	if (servo_channel < 0 || servo_channel >= bufLen) {
		return -1;
	}

	return reader->ADC_buf[servo_channel];
}


void updateAngles(servoAngleReader* reader) {
	for (int i = 0; i < 13; i++) {
		reader->servoAngles[i] = 180.0 * reader->ADC_buf[i] / 4096.0;
	}
}


double getAngleValue(servoAngleReader* reader, uint8_t servo_channel) {
	if (servo_channel < 0 || servo_channel >= bufLen) {
		return -1;
	}

	return reader->servoAngles[servo_channel];
}

double getBatteryVoltage(servoAngleReader* reader) {
	return (reader->ADC_buf[BATT_CH] / 4096.0) * (3.3 *  2); // Multiply by two for the battery division ratio
}
#endif
