#ifndef _MOVEMENT_CONTROLLER_H_
#define _MOVEMENT_CONTROLLER_H_

#include "state_controller.h"
#include "inverse_kinematics.h"
#include "MPU6050.h"

#define X_OFFSET 0
#define Y_OFFSET 14

typedef struct {
    uint16_t movement_counter;
    movement_state executing_state;

    float LF_Y_stab;
    float RF_Y_stab;
    float LB_Y_stab;
    float RB_Y_stab;

} movement_controller;

// Square Wave Array
float square_wave[] = {
   1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f,
   1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f,
   1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f,
   0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f,
   0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f,
   0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f,
   0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f,
   0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f,
   0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f,
   0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f,
   0.000000f
};

// Triangle Wave Array
float triangle_wave[] = {
   -1.000000f, -0.920000f, -0.840000f, -0.760000f, -0.680000f, -0.600000f, -0.520000f, -0.440000f, -0.360000f, -0.280000f,
   -0.200000f, -0.120000f, -0.040000f, 0.040000f, 0.120000f, 0.200000f, 0.280000f, 0.360000f, 0.440000f, 0.520000f,
   0.600000f, 0.680000f, 0.760000f, 0.840000f, 0.920000f, 1.000000f, 0.973684f, 0.947368f, 0.921053f, 0.894737f,
   0.868421f, 0.842105f, 0.815789f, 0.789474f, 0.763158f, 0.736842f, 0.710526f, 0.684211f, 0.657895f, 0.631579f,
   0.605263f, 0.578947f, 0.552632f, 0.526316f, 0.500000f, 0.473684f, 0.447368f, 0.421053f, 0.394737f, 0.368421f,
   0.342105f, 0.315789f, 0.289474f, 0.263158f, 0.236842f, 0.210526f, 0.184211f, 0.157895f, 0.131579f, 0.105263f,
   0.078947f, 0.052632f, 0.026316f, 0.000000f, -0.026316f, -0.052632f, -0.078947f, -0.105263f, -0.131579f, -0.157895f,
   -0.184211f, -0.210526f, -0.236842f, -0.263158f, -0.289474f, -0.315789f, -0.342105f, -0.368421f, -0.394737f, -0.421053f,
   -0.447368f, -0.473684f, -0.500000f, -0.526316f, -0.552632f, -0.578947f, -0.605263f, -0.631579f, -0.657895f, -0.684211f,
   -0.710526f, -0.736842f, -0.763158f, -0.789474f, -0.815789f, -0.842105f, -0.868421f, -0.894737f, -0.921053f, -0.947368f,
   -0.973684f
};


// float square_wave[] = {
//     1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f,
//     1.000000f, 1.000000f, 1.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f,
//     0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f,
//     0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f,
//     0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f
// };

// // Triangle Wave Array
// float triangle_wave[] = {
//     -1.000000f, -0.846154f, -0.692308f, -0.538462f, -0.384615f, -0.230769f, -0.076923f, 0.076923f, 0.230769f, 0.384615f,
//     0.538462f, 0.692308f, 0.846154f, 1.000000f, 0.947368f, 0.894737f, 0.842105f, 0.789474f, 0.736842f, 0.684211f,
//     0.631579f, 0.578947f, 0.526316f, 0.473684f, 0.421053f, 0.368421f, 0.315789f, 0.263158f, 0.210526f, 0.157895f,
//     0.105263f, 0.052632f, 0.000000f, -0.052632f, -0.105263f, -0.157895f, -0.210526f, -0.263158f, -0.315789f, -0.368421f,
//     -0.421053f, -0.473684f, -0.526316f, -0.578947f, -0.631579f, -0.684211f, -0.736842f, -0.789474f, -0.842105f, -0.894737f
// };


// Call this in the 100Hz loop!
// STANDING,
// GYRATING,
// WALKING,
// SITTING,
// LOAFING,
void updateMovements(movement_controller* mc, state_controller* sc, servo_set* _servo_set, leg_position* LB, leg_position* RB, leg_position* LF, leg_position* RF) {
    // Only start at the beginning of a walking cycle
    // if (mc->movement_counter == 0) {
    if (true) {
        mc->executing_state = sc->current_move_state;
    }

    float acc_x, acc_y, acc_z = 0;

	switch (mc->executing_state) {
		case (WALKING):
			setDesiredPosition(LB, X_OFFSET - triangle_wave[(mc->movement_counter + 0) % 100] * (float)sc->joystick_x , Y_OFFSET - square_wave[(mc->movement_counter + 0) % 100] * 4 , 3.9 - triangle_wave[(mc->movement_counter + 0) % 100] * (float)sc->joystick_y / 2.5);
			setDesiredPosition(RB, X_OFFSET - triangle_wave[(mc->movement_counter + 25) % 100] * (float)sc->joystick_x , Y_OFFSET - square_wave[(mc->movement_counter + 25) % 100] * 4 , 3.9 - triangle_wave[(mc->movement_counter + 25) % 100] * (float)sc->joystick_y / 2.5);
			setDesiredPosition(LF, X_OFFSET - triangle_wave[(mc->movement_counter + 50) % 100] * (float)sc->joystick_x, Y_OFFSET - square_wave[(mc->movement_counter + 50) % 100] * 4 , 3.9 - triangle_wave[(mc->movement_counter + 50) % 100] * (float)sc->joystick_y / 2.5);
			setDesiredPosition(RF, X_OFFSET - triangle_wave[(mc->movement_counter + 75) % 100] * (float)sc->joystick_x, Y_OFFSET - square_wave[(mc->movement_counter + 75) % 100] * 4 , 3.9 - triangle_wave[(mc->movement_counter + 75) % 100] * (float)sc->joystick_y / 2.5);

            break;
        
        case (STANDING):
//			 MPU6050_Read_Accel(&acc_x, &acc_y, &acc_z);
//			 computeStanding(mc, acc_x, acc_y, acc_z);

            setDesiredPosition(LB, X_OFFSET, Y_OFFSET , 3.9);
            setDesiredPosition(RB, X_OFFSET, Y_OFFSET , 3.9);
            setDesiredPosition(LF, X_OFFSET, Y_OFFSET , 3.9);
            setDesiredPosition(RF, X_OFFSET, Y_OFFSET , 3.9);

            break;
//
//        case (GYRATING):
//            // Using the standing function to gyrate
//            // The  joystick values are fed into the IMU input of the function
//            computeStanding(mc, 100 + 10.0 * (sc->joystick_x) * 3, 0, 0);
//
//            setDesiredPosition(LB, X_OFFSET, Y_OFFSET + mc->LB_Y_stab, 3.9);
//            setDesiredPosition(RB, X_OFFSET, Y_OFFSET + mc->RB_Y_stab, 3.9);
//            setDesiredPosition(LF, X_OFFSET, Y_OFFSET + mc->LF_Y_stab, 3.9);
//            setDesiredPosition(RF, X_OFFSET, Y_OFFSET + mc->RF_Y_stab, 3.9);
//
//            break;
//
//        case (SITTING):
//            setDesiredPosition(LB, X_OFFSET, Y_OFFSET - 4, 3.9);
//            setDesiredPosition(RB, X_OFFSET, Y_OFFSET - 4, 3.9);
//            setDesiredPosition(LF, X_OFFSET, Y_OFFSET, 3.9);
//            setDesiredPosition(RF, X_OFFSET, Y_OFFSET, 3.9);

            break;

        case (LOAFING):
            break;
	}

    // Increment movement states till 1000
    if (mc->movement_counter >= 99) {
        mc->movement_counter = 0;
    }
    else {
        mc->movement_counter++;
    }
}


void computeStanding(movement_controller* mc, float acc_x, float acc_y, float acc_z) {
    // Correcting the pitching movement
    if (acc_x < 100) {
        // If pitched down, lift the front up and push the back down
        mc->LF_Y_stab += 0.001;
        mc->RF_Y_stab += 0.001;

        mc->LB_Y_stab -= 0.001;
        mc->RB_Y_stab -= 0.001;
    }
    else {
        // If pitched up
        mc->LF_Y_stab -= 0.001;
        mc->RF_Y_stab -= 0.001;

        mc->LB_Y_stab += 0.001;
        mc->RB_Y_stab += 0.001;
    }


    // Clamping the adjustment values to -2 to 2
    if (mc->LB_Y_stab > 2 || mc->LB_Y_stab < -2) {
        if (mc->LB_Y_stab > 2) {
            mc->LB_Y_stab = 2;
        }
        else {
            mc->LB_Y_stab = -2;
        }
    }

    if (mc->RB_Y_stab > 2 || mc->RB_Y_stab < -2) {
        if (mc->RB_Y_stab > 2) {
            mc->RB_Y_stab = 2;
        }
        else {
            mc->RB_Y_stab = -2;
        }
    }

    if (mc->LF_Y_stab > 2 || mc->LF_Y_stab < -2) {
        if (mc->LF_Y_stab > 2) {
            mc->LF_Y_stab = 2;
        }
        else {
            mc->LF_Y_stab = -2;
        }
    }

    if (mc->RF_Y_stab > 2 || mc->RF_Y_stab < -2) {
        if (mc->RF_Y_stab > 2) {
            mc->RF_Y_stab = 2;
        }
        else {
            mc->RF_Y_stab = -2;
        }
    }

    return;
}

#endif
