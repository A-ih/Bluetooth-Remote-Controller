#ifndef _FACE_CONTROLLER_H_
#define _FACE_CONTROLLER_H_

#include "state_controller.h"
#include "bitmaps.h"
#include "lcd.h"

#define SKIN_COLOR  0x0000
#define EYE_COLOR   0xC440 

/*
    NORMAL, // Default state
    SLEEPY, // Low battery state < 4.5V
    SAD,    // Did not interact after 1 minute
    ANGRY,  // Touched belly
    HAPPY,  // Pet head
    ALARM,  // Triggered watchdog
    DEAD,   // Low battery, < 4V
*/


typedef struct {
    multimedia_state last_state;    // To check if a screen update is necessary
    uint8_t animation_counter;      // 0 to 3 for the sleep animation
} face_controller;

// Call this in the 1s loop!
void updateFace(face_controller* fc, state_controller* sc) {
    if (fc->last_state != sc->current_mm_state) {
        fc->last_state = sc->current_mm_state;

        switch (sc->current_mm_state) {
            case (NORMAL):
                LCD_DrawFace(0, 0, face_normal, SKIN_COLOR, EYE_COLOR);
                break;

            case (SLEEPY):
                if (fc->animation_counter == 0) {
                    (fc->animation_counter)++;
                    LCD_DrawFace(0, 0, face_sleep_1, SKIN_COLOR, EYE_COLOR);
                }
                else if (fc->animation_counter == 1) {
                    (fc->animation_counter)++;
                    LCD_DrawFace(0, 0, face_sleep_2, SKIN_COLOR, EYE_COLOR);
                }
                else if (fc->animation_counter == 2) {
                    (fc->animation_counter)++;
                    LCD_DrawFace(0, 0, face_sleep_3, SKIN_COLOR, EYE_COLOR);
                }
                else if (fc->animation_counter == 3) {
                    (fc->animation_counter) = 0;
                    LCD_DrawFace(0, 0, face_sleep_4, SKIN_COLOR, EYE_COLOR);
                }
                break;
            
            case (SAD):
                LCD_DrawFace(0, 0, face_sad, SKIN_COLOR, EYE_COLOR);
                break;

            case (ANGRY):
                LCD_DrawFace(0, 0, face_angry, SKIN_COLOR, EYE_COLOR);
                break;

            case (HAPPY):
                LCD_DrawFace(0, 0, face_happy, SKIN_COLOR, EYE_COLOR);
                break;

            case (ALARM):
                LCD_DrawFace(0, 0, face_angry, SKIN_COLOR, EYE_COLOR);
                break;
        
            case (DEAD):
                LCD_DrawFace(0, 0, face_dead, SKIN_COLOR, EYE_COLOR);
                break;
        }
    }
}

#endif