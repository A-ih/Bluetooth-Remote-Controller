/*
 * servo_pid_set.h
 *
 *  Created on: Apr 28, 2025
 *      Author: phili
 */

#ifndef INC_SERVO_PID_SET_H_
#define INC_SERVO_PID_SET_H_

#include "pid.h"
#include "pca9685.h"
#include "main.h"

#define LB_TIBIA_ADC 1
#define LB_FEMUR_ADC 0
#define LB_SHOUL_ADC 7

#define LF_TIBIA_ADC 11
#define LF_FEMUR_ADC 10
#define LF_SHOUL_ADC 8

#define RF_TIBIA_ADC 3
#define RF_FEMUR_ADC 2
#define RF_SHOUL_ADC 5

#define RB_TIBIA_ADC 4
#define RB_FEMUR_ADC 12
#define RB_SHOUL_ADC 6




#define LB_TIBIA_F_2 8
#define LB_TIBIA_B_2 9
#define LB_FEMUR_F_2 13
#define LB_FEMUR_B_2 12
#define LB_SHOUL_F_2 11
#define LB_SHOUL_B_2 10

#define LF_TIBIA_F_1 0
#define LF_TIBIA_B_1 8
#define LF_FEMUR_F_1 1	
#define LF_FEMUR_B_1 2	
#define LF_SHOUL_F_1 3
#define LF_SHOUL_B_1 9

#define RF_TIBIA_F_1 10
#define RF_TIBIA_B_1 11
#define RF_FEMUR_F_1 15
#define RF_FEMUR_B_1 14
#define RF_SHOUL_F_1 13
#define RF_SHOUL_B_1 12

#define RB_TIBIA_F_1 7
#define RB_TIBIA_B_1 6
#define RB_FEMUR_F_2 15
#define RB_FEMUR_B_2 14
#define RB_SHOUL_F_1 4
#define RB_SHOUL_B_1 5

#define NECK_CH 9

#define bufLen 13


typedef struct {
	servoAngleReader* servo_reader;

	PID_TypeDef* LB_Tibia_PID;
	PID_TypeDef* LB_Femur_PID;
	PID_TypeDef* LB_Shoul_PID;
	PID_TypeDef* LF_Tibia_PID;
	PID_TypeDef* LF_Femur_PID;
	PID_TypeDef* LF_Shoul_PID;
	PID_TypeDef* RF_Tibia_PID;
	PID_TypeDef* RF_Femur_PID;
	PID_TypeDef* RF_Shoul_PID;
	PID_TypeDef* RB_Tibia_PID;
	PID_TypeDef* RB_Femur_PID;
	PID_TypeDef* RB_Shoul_PID;

	double setpoints [bufLen];
	double angles [bufLen];
	double outputs [bufLen];

	double filtered_output [8][bufLen];

	pca9685_handle_t* pwm_out_1;
	pca9685_handle_t* pwm_out_2;

	I2C_HandleTypeDef* hi2c_1;
	I2C_HandleTypeDef* hi2c_2;

	uint8_t smoothing_counter;
} servo_set;



void updateAnglesArray(servo_set* _servo_set, double angles[bufLen]) {
	for (int i = 0; i < bufLen; i++) {
		_servo_set->angles[i] = angles[i];
	}
}


void updateSetpointsArray(servo_set* _servo_set, double setpoints[bufLen]) {
	for (int i = 0; i < bufLen; i++) {
		_servo_set->setpoints[i] = setpoints[i];
	}
}


void updateOutputsArray(servo_set* _servo_set, double outputs[bufLen]) {
	for (int i = 0; i < bufLen; i++) {
		_servo_set->outputs[i] = outputs[i];
	}
}

void initFilteredOutputArray(servo_set* _servo_set) {
	for (int i = 0; i < 8; i++) {
		for (int j = 0; j < bufLen; j++) {
			_servo_set->filtered_output[i][j] = 0;
		}
	}
}


uint8_t pidInit(servo_set* _servo_set) {
	double zeros[bufLen] = {0};

	// Initalizing the three arrays
	updateSetpointsArray(_servo_set, zeros);
	updateAnglesArray(_servo_set, zeros);
	updateOutputsArray(_servo_set, zeros);
	initFilteredOutputArray(_servo_set);

	_servo_set->smoothing_counter = 0;
	// 1.5 0 0.006 Works!
	// 0.2, 0.1, 0.0
	// 1, 0.1, 0.05
	// 10, 0.1, 0.5
	// 100, 0.1, 5
	// 500, 1, 25
	// 1500, 2, 20

	// WORKS: 100, 0, 0.08
	PID(_servo_set->LB_Tibia_PID, &(_servo_set->angles[LB_TIBIA_ADC]), &(_servo_set->outputs[LB_TIBIA_ADC]), &(_servo_set->setpoints[LB_TIBIA_ADC]), 85, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);
	PID(_servo_set->LB_Femur_PID, &(_servo_set->angles[LB_FEMUR_ADC]), &(_servo_set->outputs[LB_FEMUR_ADC]), &(_servo_set->setpoints[LB_FEMUR_ADC]), 85, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);
	PID(_servo_set->LB_Shoul_PID, &(_servo_set->angles[LB_SHOUL_ADC]), &(_servo_set->outputs[LB_SHOUL_ADC]), &(_servo_set->setpoints[LB_SHOUL_ADC]), 170, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);
	PID(_servo_set->LF_Tibia_PID, &(_servo_set->angles[LF_TIBIA_ADC]), &(_servo_set->outputs[LF_TIBIA_ADC]), &(_servo_set->setpoints[LF_TIBIA_ADC]), 85, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);
	PID(_servo_set->LF_Femur_PID, &(_servo_set->angles[LF_FEMUR_ADC]), &(_servo_set->outputs[LF_FEMUR_ADC]), &(_servo_set->setpoints[LF_FEMUR_ADC]), 85, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);
	PID(_servo_set->LF_Shoul_PID, &(_servo_set->angles[LF_SHOUL_ADC]), &(_servo_set->outputs[LF_SHOUL_ADC]), &(_servo_set->setpoints[LF_SHOUL_ADC]), 170, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);
	PID(_servo_set->RF_Tibia_PID, &(_servo_set->angles[RF_TIBIA_ADC]), &(_servo_set->outputs[RF_TIBIA_ADC]), &(_servo_set->setpoints[RF_TIBIA_ADC]), 85, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);
	PID(_servo_set->RF_Femur_PID, &(_servo_set->angles[RF_FEMUR_ADC]), &(_servo_set->outputs[RF_FEMUR_ADC]), &(_servo_set->setpoints[RF_FEMUR_ADC]), 85, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);
	PID(_servo_set->RF_Shoul_PID, &(_servo_set->angles[RF_SHOUL_ADC]), &(_servo_set->outputs[RF_SHOUL_ADC]), &(_servo_set->setpoints[RF_SHOUL_ADC]), 170, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);
	PID(_servo_set->RB_Tibia_PID, &(_servo_set->angles[RB_TIBIA_ADC]), &(_servo_set->outputs[RB_TIBIA_ADC]), &(_servo_set->setpoints[RB_TIBIA_ADC]), 85, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);
	PID(_servo_set->RB_Femur_PID, &(_servo_set->angles[RB_FEMUR_ADC]), &(_servo_set->outputs[RB_FEMUR_ADC]), &(_servo_set->setpoints[RB_FEMUR_ADC]), 85, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);
	PID(_servo_set->RB_Shoul_PID, &(_servo_set->angles[RB_SHOUL_ADC]), &(_servo_set->outputs[RB_SHOUL_ADC]), &(_servo_set->setpoints[RB_SHOUL_ADC]), 170, 1, 0, _PID_P_ON_E, _PID_CD_DIRECT);

	PID_Init(_servo_set->LB_Tibia_PID);
	PID_Init(_servo_set->LB_Femur_PID);
	PID_Init(_servo_set->LB_Shoul_PID);
	PID_Init(_servo_set->LF_Tibia_PID);
	PID_Init(_servo_set->LF_Femur_PID);
	PID_Init(_servo_set->LF_Shoul_PID);
	PID_Init(_servo_set->RF_Tibia_PID);
	PID_Init(_servo_set->RF_Femur_PID);
	PID_Init(_servo_set->RF_Shoul_PID);
	PID_Init(_servo_set->RB_Tibia_PID);
	PID_Init(_servo_set->RB_Femur_PID);
	PID_Init(_servo_set->RB_Shoul_PID);

	PID_SetSampleTime(_servo_set->LB_Tibia_PID, 1);
	PID_SetSampleTime(_servo_set->LB_Femur_PID, 1);
	PID_SetSampleTime(_servo_set->LB_Shoul_PID, 1);
	PID_SetSampleTime(_servo_set->LF_Tibia_PID, 1);
	PID_SetSampleTime(_servo_set->LF_Femur_PID, 1);
	PID_SetSampleTime(_servo_set->LF_Shoul_PID, 1);
	PID_SetSampleTime(_servo_set->RF_Tibia_PID, 1);
	PID_SetSampleTime(_servo_set->RF_Femur_PID, 1);
	PID_SetSampleTime(_servo_set->RF_Shoul_PID, 1);
	PID_SetSampleTime(_servo_set->RB_Tibia_PID, 1);
	PID_SetSampleTime(_servo_set->RB_Femur_PID, 1);
	PID_SetSampleTime(_servo_set->RB_Shoul_PID, 1);

	PID_SetMode(_servo_set->LB_Tibia_PID, _PID_MODE_AUTOMATIC);
	PID_SetMode(_servo_set->LB_Femur_PID, _PID_MODE_AUTOMATIC);
	PID_SetMode(_servo_set->LB_Shoul_PID, _PID_MODE_AUTOMATIC);
	PID_SetMode(_servo_set->LF_Tibia_PID, _PID_MODE_AUTOMATIC);
	PID_SetMode(_servo_set->LF_Femur_PID, _PID_MODE_AUTOMATIC);
	PID_SetMode(_servo_set->LF_Shoul_PID, _PID_MODE_AUTOMATIC);
	PID_SetMode(_servo_set->RF_Tibia_PID, _PID_MODE_AUTOMATIC);
	PID_SetMode(_servo_set->RF_Femur_PID, _PID_MODE_AUTOMATIC);
	PID_SetMode(_servo_set->RF_Shoul_PID, _PID_MODE_AUTOMATIC);
	PID_SetMode(_servo_set->RB_Tibia_PID, _PID_MODE_AUTOMATIC);
	PID_SetMode(_servo_set->RB_Femur_PID, _PID_MODE_AUTOMATIC);
	PID_SetMode(_servo_set->RB_Shoul_PID, _PID_MODE_AUTOMATIC);

	PID_SetOutputLimits(_servo_set->LB_Tibia_PID, -4095, 4095);
	PID_SetOutputLimits(_servo_set->LB_Femur_PID, -4095, 4095);
	PID_SetOutputLimits(_servo_set->LB_Shoul_PID, -4095, 4095);
	PID_SetOutputLimits(_servo_set->LF_Tibia_PID, -4095, 4095);
	PID_SetOutputLimits(_servo_set->LF_Femur_PID, -4095, 4095);
	PID_SetOutputLimits(_servo_set->LF_Shoul_PID, -4095, 4095);
	PID_SetOutputLimits(_servo_set->RF_Tibia_PID, -4095, 4095);
	PID_SetOutputLimits(_servo_set->RF_Femur_PID, -4095, 4095);
	PID_SetOutputLimits(_servo_set->RF_Shoul_PID, -4095, 4095);
	PID_SetOutputLimits(_servo_set->RB_Tibia_PID, -4095, 4095);
	PID_SetOutputLimits(_servo_set->RB_Femur_PID, -4095, 4095);
	PID_SetOutputLimits(_servo_set->RB_Shoul_PID, -4095, 4095);


	if (!(pca9685_init(_servo_set->pwm_out_1))) {
		return 0;
	}

	if (!(pca9685_init(_servo_set->pwm_out_2))) {
		return 0;
	}

	pca9685_wakeup(_servo_set->pwm_out_1);
	pca9685_wakeup(_servo_set->pwm_out_2);

	pca9685_set_pwm_frequency(_servo_set->pwm_out_1, 1526);
	pca9685_set_pwm_frequency(_servo_set->pwm_out_2, 1526);

	return 1;
}


void updateServo(servo_set* _servo_set) {
	if (_servo_set->smoothing_counter >= 0) {
		_servo_set->smoothing_counter = 0;
	}
	else {
		_servo_set->smoothing_counter++;
	}
	// Computing the output, then output to the PWM driver over I2C
	PID_Compute(_servo_set->LB_Tibia_PID);
	PID_Compute(_servo_set->LB_Femur_PID);
	PID_Compute(_servo_set->LB_Shoul_PID);

	PID_Compute(_servo_set->LF_Tibia_PID);
	PID_Compute(_servo_set->LF_Femur_PID);
	PID_Compute(_servo_set->LF_Shoul_PID);

	PID_Compute(_servo_set->RF_Tibia_PID);
	PID_Compute(_servo_set->RF_Femur_PID);
	PID_Compute(_servo_set->RF_Shoul_PID);

	PID_Compute(_servo_set->RB_Tibia_PID);
	PID_Compute(_servo_set->RB_Femur_PID);
	PID_Compute(_servo_set->RB_Shoul_PID);


	// for (int i = 0; i < bufLen; i++) {
	// 	_servo_set->filtered_output[_servo_set->smoothing_counter][i] = _servo_set->outputs[i];
	// }
	
	// THIS PART IS FUCKY
	// float averaged_output[bufLen] = {0};

	// for (int i = 0; i < bufLen; i++) {
	// 	averaged_output[i] = (_servo_set->filtered_output[0][i]);
	// 	// averaged_output[i] = (_servo_set->filtered_output[0][i] + _servo_set->filtered_output[1][i] + _servo_set->filtered_output[2][i] + _servo_set->filtered_output[3][i]) / 4;
	// 						// _servo_set->filtered_output[4][i] + _servo_set->filtered_output[5][i] + _servo_set->filtered_output[6][i] + _servo_set->filtered_output[7][i]) / 8.0;
	// }
	
	unsigned int onehot_F_output [bufLen] = {0};
	unsigned int onehot_B_output [bufLen] = {0};


	for (int i = 0; i < bufLen; i++) {
		if (_servo_set->outputs[i] >= 0) {
			onehot_F_output[i] = _servo_set->outputs[i];
			onehot_B_output[i] = 0;
		}
		else {
			onehot_F_output[i] = 0;
			onehot_B_output[i] = (unsigned int)(-_servo_set->outputs[i]);
		}
	}

	// PWM Controller 1
	pca9685_set_channel_pwm_times((_servo_set->pwm_out_1), RF_TIBIA_F_1, 0, (onehot_F_output[RF_TIBIA_ADC]));
	pca9685_set_channel_pwm_times((_servo_set->pwm_out_1), RF_TIBIA_B_1, 0, (onehot_B_output[RF_TIBIA_ADC]));
	pca9685_set_channel_pwm_times((_servo_set->pwm_out_1), RF_FEMUR_F_1, 0, (onehot_F_output[RF_FEMUR_ADC]));
	pca9685_set_channel_pwm_times((_servo_set->pwm_out_1), RF_FEMUR_B_1, 0, (onehot_B_output[RF_FEMUR_ADC]));
	pca9685_set_channel_pwm_times((_servo_set->pwm_out_1), RF_SHOUL_F_1, 0, (onehot_F_output[RF_SHOUL_ADC]));
	pca9685_set_channel_pwm_times((_servo_set->pwm_out_1), RF_SHOUL_B_1, 0, (onehot_B_output[RF_SHOUL_ADC]));
	pca9685_set_channel_pwm_times((_servo_set->pwm_out_1), LF_TIBIA_F_1, 0, (onehot_F_output[LF_TIBIA_ADC]));
	pca9685_set_channel_pwm_times((_servo_set->pwm_out_1), LF_TIBIA_B_1, 0, (onehot_B_output[LF_TIBIA_ADC]));
	pca9685_set_channel_pwm_times((_servo_set->pwm_out_1), LF_FEMUR_F_1, 0, (onehot_F_output[LF_FEMUR_ADC]));
	pca9685_set_channel_pwm_times((_servo_set->pwm_out_1), LF_FEMUR_B_1, 0, (onehot_B_output[LF_FEMUR_ADC]));
	pca9685_set_channel_pwm_times((_servo_set->pwm_out_1), LF_SHOUL_F_1, 0, (onehot_F_output[LF_SHOUL_ADC]));
	pca9685_set_channel_pwm_times((_servo_set->pwm_out_1), LF_SHOUL_B_1, 0, (onehot_B_output[LF_SHOUL_ADC]));

//	pca9685_wakeup((_servo_set->pwm_out_1));

	pca9685_set_channel_pwm_times((_servo_set->pwm_out_1), RB_TIBIA_F_1, 0, (onehot_F_output[RB_TIBIA_ADC]));
	pca9685_set_channel_pwm_times((_servo_set->pwm_out_1), RB_TIBIA_B_1, 0, (onehot_B_output[RB_TIBIA_ADC]));
	pca9685_set_channel_pwm_times((_servo_set->pwm_out_1), RB_SHOUL_F_1, 0, (onehot_F_output[RB_SHOUL_ADC]));
	pca9685_set_channel_pwm_times((_servo_set->pwm_out_1), RB_SHOUL_B_1, 0, (onehot_B_output[RB_SHOUL_ADC]));

//	pca9685_wakeup((_servo_set->pwm_out_2));
	// PWM Controller 2
	pca9685_set_channel_pwm_times((_servo_set->pwm_out_2), LB_TIBIA_F_2, 0, (onehot_F_output[LB_TIBIA_ADC]));
	pca9685_set_channel_pwm_times((_servo_set->pwm_out_2), LB_TIBIA_B_2, 0, (onehot_B_output[LB_TIBIA_ADC]));
	pca9685_set_channel_pwm_times((_servo_set->pwm_out_2), LB_FEMUR_F_2, 0, (onehot_F_output[LB_FEMUR_ADC]));
	pca9685_set_channel_pwm_times((_servo_set->pwm_out_2), LB_FEMUR_B_2, 0, (onehot_B_output[LB_FEMUR_ADC]));
	pca9685_set_channel_pwm_times((_servo_set->pwm_out_2), LB_SHOUL_F_2, 0, (onehot_F_output[LB_SHOUL_ADC])); 
	pca9685_set_channel_pwm_times((_servo_set->pwm_out_2), LB_SHOUL_B_2, 0, (onehot_B_output[LB_SHOUL_ADC]));

	pca9685_set_channel_pwm_times((_servo_set->pwm_out_2), RB_FEMUR_F_2, 0, (onehot_F_output[RB_FEMUR_ADC]));
	pca9685_set_channel_pwm_times((_servo_set->pwm_out_2), RB_FEMUR_B_2, 0, (onehot_B_output[RB_FEMUR_ADC]));
}


#endif /* INC_SERVO_PID_SET_H_ */
