#ifndef _INVERSE_KINEMATICS_H_
#define _INVERSE_KINEMATICS_H_

#include "math.h"
#include "stm32f4xx_hal.h"

#define PI 3.141592653589793
#define TWO_PI 6.28318530718
#define PI_OVER_TWO 1.57079632679

typedef enum {
    LB,
    RB,
    LF,
    RF
} leg_set;

typedef struct {
    leg_set leg;

    float x;
    float y;
    float z;

    double shoul_omega;
    double femur_theta;
    double tibia_epsilon;
} leg_position;

void inverse_kinematics(leg_position* leg) {
    float D, G, phi;

    // Outputs
    float omega, theta, epsilon;    // Direct computed angle for shoulder, femur and tibia respectively, before correction
    float cr_omega, cr_theta, cr_epsilon;    // Corrected values

    D = sqrtf(powf(leg->z, 2) + pow(leg->y, 2) - 3.9*3.9);

    G = sqrtf(pow(D, 2) + pow((leg->x), 2));

    phi = acos((G * G - 199.92) / (-196.89));

    omega = atan((leg->z) / (leg->y)) + atan(D / 3.9);

    theta = atan((leg->x) / D) + atan(10.83 * sinf(phi) / G);

    epsilon = theta + phi;

    // Applying hardcoded angular correction for these shitass servos
    switch(leg->leg) {
        case (LB):
            cr_omega =  omega;
            cr_theta = (PI - theta - 1.471138) * 1.14;
            cr_epsilon = (PI - epsilon - 0.4101524) * 1 + PI_OVER_TWO;
            break;
        
        case (RB):
            cr_omega = omega;
            cr_theta = (PI - theta - 1.552994) * 1.143;
            cr_epsilon = (PI - epsilon - 0.52307518) * 1.245 + PI_OVER_TWO;
            break;

        case (LF):
            cr_omega = omega;
            cr_theta = (PI - theta - 1.6015141) * 1.339;
            cr_epsilon = (PI - epsilon - 0.4701917) * 1.145 + PI_OVER_TWO;
            break;

        case (RF):
            cr_omega = omega;
            cr_theta = (PI - theta - 1.5961036) * 1.273;
            cr_epsilon = (PI - epsilon - 0.4977679) * 1.105 + PI_OVER_TWO;
            break;
    }

    leg->shoul_omega = cr_omega;
    leg->femur_theta = cr_theta;
    leg->tibia_epsilon = cr_epsilon;
}


// Call this function to update and calculate the legs position
void setDesiredPosition(leg_position* leg, float x, float height_y, float z) {
	leg->x = x;
	leg->y = height_y;
	leg->z = z;

	inverse_kinematics(leg);
}


// Call this function to commit the new angles to the PID controllers
void updateLeg(leg_position* leg, servo_set* _servo_set) {
    switch (leg->leg) {
        case (LB):
            (_servo_set->setpoints)[LB_TIBIA_ADC] = leg->tibia_epsilon * 180 / PI;
        	(_servo_set->setpoints)[LB_FEMUR_ADC] = leg->femur_theta * 180 / PI;
        	(_servo_set->setpoints)[LB_SHOUL_ADC] = leg->shoul_omega * 180 / PI;
            break;

        case (RB):
			(_servo_set->setpoints)[RB_TIBIA_ADC] = 180 - (leg->tibia_epsilon * 180 / PI);
			(_servo_set->setpoints)[RB_FEMUR_ADC] = 180 - (leg->femur_theta * 180 / PI);
			(_servo_set->setpoints)[RB_SHOUL_ADC] = leg->shoul_omega * 180 / PI;
			break;

        case (LF):
            (_servo_set->setpoints)[LF_TIBIA_ADC] = leg->tibia_epsilon * 180 / PI;
        	(_servo_set->setpoints)[LF_FEMUR_ADC] = leg->femur_theta * 180 / PI;
        	(_servo_set->setpoints)[LF_SHOUL_ADC] = leg->shoul_omega * 180 / PI;
            break;

        case (RF):
			(_servo_set->setpoints)[RF_TIBIA_ADC] = 180 - (leg->tibia_epsilon * 180 / PI);
			(_servo_set->setpoints)[RF_FEMUR_ADC] = 180 - (leg->femur_theta * 180 / PI);
			(_servo_set->setpoints)[RF_SHOUL_ADC] = leg->shoul_omega * 180 / PI;
			break;
    }
}


#endif
