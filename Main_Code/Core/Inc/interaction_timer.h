/*
 * rtc.h
 *
 *  Created on: May 4, 2025
 *      Author: phili
 */

#ifndef INC_INTERACTION_TIMER_H_
#define INC_INTERACTION_TIMER_H_


// If set to 1 Min, 0 Hour, 0 Day, state will be SAD 
// if current time - last time is >= threshold
#define SAD_THRESHOLD_MIN	1
#define SAD_THRESHOLD_HOUR 	0
#define SAD_THRESHOLD_DAY	0



typedef struct {
	RTC_TimeTypeDef LastTime;	// Last time of interaction
	RTC_DateTypeDef LastDate;	// Last date of interaction

	uint8_t sadThresholdMin;
	uint8_t sadThresholdHour;
	uint8_t sadThresholdDay;

	RTC_HandleTypeDef* hrtc;
} interaction_timer;




void interactUpdate(interaction_timer* rtc_handler) {
	HAL_RTC_GetTime(rtc_handler->hrtc, &rtc_handler->LastTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(rtc_handler->hrtc, &rtc_handler->LastDate, RTC_FORMAT_BIN);
}


uint8_t checkIfSad(interaction_timer* rtc_handler) {
	RTC_TimeTypeDef CurrTime;	// Last time of interaction
	RTC_DateTypeDef CurrDate;	// Last date of interaction

	HAL_RTC_GetTime(rtc_handler->hrtc, &CurrTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(rtc_handler->hrtc, &CurrDate, RTC_FORMAT_BIN);


	if ((CurrDate.Date - rtc_handler->LastDate.Date) > rtc_handler->sadThresholdDay) {
		return 1;
	}
	else if ((CurrTime.Hours - rtc_handler->LastTime.Hours) > rtc_handler->sadThresholdHour) {
		return 1;
	}
	else if ((CurrTime.Minutes - rtc_handler->LastTime.Minutes) > rtc_handler->sadThresholdMin) {
		return 1;
	}
	else {
		return 0;
	}
}


#endif /* INC_INTERACTION_TIMER_H_ */
