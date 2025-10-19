/*
 * watchdog_function.h
 *
 *  Created on: May 8, 2025
 *      Author: phili
 */

#ifndef INC_WATCHDOG_FUNCTION_H_
#define INC_WATCHDOG_FUNCTION_H_

#include "state_controller.h"

//#define DISTANCE_THRESHOLD 500

void watchdog_init(I2C_HandleTypeDef* hi2c1) {
    //init VL53L0X
	char msgBuffer[52];
	for (uint8_t i = 0; i < 52; i++) {
		msgBuffer[i] = ' ';
	}

	// Initialise the VL53L0X
	statInfo_t_VL53L0X distanceStr;
	initVL53L0X(1, &hi2c1);

	// Configure the sensor for high accuracy and speed in 20 cm.
	setSignalRateLimit(20);
	setVcselPulsePeriod(VcselPeriodPreRange, 10);
	setVcselPulsePeriod(VcselPeriodFinalRange, 14);
	setMeasurementTimingBudget(300 * 1000UL);


	uint16_t distance;

    distance = readRangeSingleMillimeters(&distanceStr);

    //sprintf(msgBuffer, "Distance: %d\r\n", distance);

    //HAL_UART_Transmit(&huart2, (uint8_t*) msgBuffer, sizeof(msgBuffer), 50);


}


void watchdogTrigger(state_controller* sc, statInfo_t_VL53L0X* distanceStr) {
    static uint16_t watchdog_distance = 0;
    static uint16_t watchdog_distance_last = 0;
    const uint16_t DISTANCE_THRESHOLD = 100; // 100mm threshold for movement detection

    // Get current distance reading
    watchdog_distance = readRangeSingleMillimeters(distanceStr);

    // If this is the first reading, just store it and return
    if (watchdog_distance_last == 0) {
        watchdog_distance_last = watchdog_distance;
        return;
    }

    // Calculate absolute difference between current and last reading
    uint16_t distance_diff = (watchdog_distance > watchdog_distance_last) ?
                            (watchdog_distance - watchdog_distance_last) :
                            (watchdog_distance_last - watchdog_distance);

    // Store current reading for next comparison
    watchdog_distance_last = watchdog_distance;

    // If distance difference is too large, set alarm state
    if (distance_diff > DISTANCE_THRESHOLD) {
        sc->current_mm_state = ALARM;
    }
}


#endif /* INC_WATCHDOG_FUNCTION_H_ */
