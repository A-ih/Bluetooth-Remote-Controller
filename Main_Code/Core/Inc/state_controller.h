#ifndef _STATE_CONTROLLER_
#define _STATE_CONTROLLER_

#include "stdint.h"
#include "stm32f4xx_hal.h"

#include "interaction_timer.h"
#include "servo_adc_read.h"

// #define CTOUCH1_PORT GPIOE
// #define CTOUCH1_PIN GPIO_PIN_12

#define CTOUCH2_PORT GPIOB
#define CTOUCH2_PIN GPIO_PIN_11

// #define CTOUCH3_PORT GPIOE
// #define CTOUCH3_PIN GPIO_PIN_15

// #define CTOUCH4_PORT GPIOE
// #define CTOUCH4_PIN GPIO_PIN_14

#define CTOUCH5_PORT GPIOB
#define CTOUCH5_PIN GPIO_PIN_10


typedef enum {
    NORMAL, // Default state
    SLEEPY, // Low battery state < 4.5V
    SAD,    // Did not interact after 1 minute
    ANGRY,  // Touched belly
    HAPPY,  // Pet head
    ALARM,  // Triggered watchdog
    DEAD,   // Low battery, < 4V
} multimedia_state;

typedef enum {
    STANDING,
    GYRATING,
    WALKING,
    SITTING,
    LOAFING,
} movement_state;


typedef enum {
    CLEAR,
    PLAY_MEOW,
    PLAY_OIIA,
    GO_LOAF,
    ENABLE_WATCHDOG,
} touchscreen_state;

typedef enum {
    JOYSTICK_PRESSED,
    JOYSTICK_RELEASED,
} joystick_button_state;




typedef struct {
    // For multimedia functions
    uint8_t     ctouch_state[5];
    float    battery_voltage_adc;
    // RTC

    touchscreen_state touchscreen_state;

    int8_t joystick_x;
    int8_t joystick_y;
    uint8_t joystick_button;
    uint8_t ts_button;

    servoAngleReader* _servo_reader;
    interaction_timer* interactionTimer;
    audioController* _audio_controller;

    multimedia_state current_mm_state;
    movement_state  current_move_state;

    UART_HandleTypeDef* bt_uart;
    statInfo_t_VL53L0X* distanceStr;
} state_controller;


// Call this in the 1000Hz loop!
// State priorities
// Dead is highest priority (1)
// Sleepy (2)
// Alarm (3)
// Angry (4)
// Happy (5)
// Sad (6)
// Normal (7)
void determineMultimediaState(state_controller* sc) {
    updateCTouch(sc);
    updateBatteryVoltage(sc);

    // Determine if Dead
    if (sc->battery_voltage_adc < 3) {
        sc->current_mm_state = DEAD;
        playAudio(sc->_audio_controller, LOW_BATTERY_AUDIO);
    }
    // Determine if sleepy
    else if (sc->battery_voltage_adc < 3.5) {
        sc->current_mm_state = SLEEPY;
    }
    // Determine if alarm
    else if (sc->ts_button == 2) {
        watchdogTrigger(sc, sc->distanceStr);
        if (sc->current_mm_state == ALARM) {
        	playAudio(sc->_audio_controller, INTRUDER_ALERT_AUDIO);
        }
    }
    // Determine if angry
    else if (sc->ctouch_state[4] == 1) {
        sc->current_mm_state = ANGRY;
    }
    // Determine if happy
    else if (sc->ctouch_state[1] == 1) {
        interactUpdate(sc->interactionTimer);
        playAudio(sc->_audio_controller, MEOW_AUDIO);
        sc->current_mm_state = HAPPY;
    }
    // Determine if sad
    else if (checkIfSad(sc->interactionTimer)) {
        sc->current_mm_state = SAD;
    }
    else {
        sc->current_mm_state = NORMAL;
    }
}



// Call this in the 1000Hz loop!
// Priority list is below
// LOAFING  (Dead)
// GYRATING (Joystick != 0, joystick_button pressed)
// WALKING  (Joystick != 0, joystick_button not pressed)
// SITTING  (Remote button)
// STANDING (Default)
void determineMovementState(state_controller* sc) {

    updateCTouch(sc);
    updateBatteryVoltage(sc);
//    watchdogTrigger(sc);

    // Determine if Loafing
    if (sc->battery_voltage_adc < 4) {
        sc->current_move_state = LOAFING;
    }
    // Determine if gyrating
    else if ((sc->joystick_x != 0 || sc->joystick_y != 0) && sc->joystick_button != 0) {
        sc->current_move_state = GYRATING;
    }
    // Determine if walking
    else if ((sc->joystick_x != 0 || sc->joystick_y != 0) && sc->joystick_button == 0) {
    	sc->current_move_state = WALKING;
    }
    // Determine if sitting
    else if (sc->ctouch_state[4] == 1) {
    	sc->current_move_state = SITTING;
    }
    else {
    	sc->current_move_state = STANDING;
    }

    if (sc->ts_button == 1) {
    	playAudio(sc->_audio_controller, OIIA_AUDIO);
    }
}   

void updateCTouch(state_controller* sc) {
//    sc->ctouch_state[0] = HAL_GPIO_ReadPin(CTOUCH1_PORT, CTOUCH1_PIN);
    sc->ctouch_state[1] = HAL_GPIO_ReadPin(CTOUCH2_PORT, CTOUCH2_PIN);
//    sc->ctouch_state[2] = HAL_GPIO_ReadPin(CTOUCH3_PORT, CTOUCH3_PIN);
//    sc->ctouch_state[3] = HAL_GPIO_ReadPin(CTOUCH4_PORT, CTOUCH4_PIN);
    sc->ctouch_state[4] = HAL_GPIO_ReadPin(CTOUCH5_PORT, CTOUCH5_PIN);
}

void updateBatteryVoltage(state_controller* sc) {
    sc->battery_voltage_adc = getBatteryVoltage(sc->_servo_reader);
}


#endif
