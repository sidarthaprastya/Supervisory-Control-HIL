#ifndef FSM_H
#define FSM_H

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define STATE_OFF_CLOSE     0
#define STATE_ACCEL_OPEN    1
#define STATE_VEL_OPEN      2
#define STATE_DECEL_OPEN    3
#define STATE_OFF_OPEN      4
#define STATE_ACCEL_CLOSE   5
#define STATE_VEL_CLOSE     6
#define STATE_DECEL_CLOSE   7


#define SEL_ACCEL   0
#define SEL_VEL     1

void fsm(int *counter, int *button, int *feed_motor, int *state, int *select, float *setpoint){
//    TickType_t xLastWakeTime;
    
    switch (*state){
        case STATE_OFF_CLOSE:
            if(*button == 1){                //tombol ditekan
                *state = STATE_ACCEL_OPEN;
                *select = 0;                //accel
                *setpoint = 1;            //2 m/s^2
                *button = 0;                //reset button
                *feed_motor = 0;            //reset feed motor
            }
            else{
                *setpoint = 0;              //setpoint kecepatan = 0
                *select = 1;
            }
            break;
        
        case STATE_ACCEL_OPEN:
            if(*feed_motor == 1){
                *state = STATE_VEL_OPEN;
                *select = 1;                //velocity
                *setpoint = 1.0;           //10 m/s
                *feed_motor = 0;
		*button = 0;
            }
            else{

            }
            break;
        
        case STATE_VEL_OPEN:
            if(*feed_motor == 1){
                *state = STATE_DECEL_OPEN;
                *select = 0;
                *setpoint = -1;
                *feed_motor = 0;
		*button = 0;
            }
            else{

            }
            break;
        
        case STATE_DECEL_OPEN:
            if(*feed_motor == 1){
                *state = STATE_OFF_OPEN;
                *select = 1;
                *setpoint = 0.0;
                *feed_motor = 0;
		*button = 0;
            }
            else{

            }
            break;
        
        case STATE_OFF_OPEN:
            if(*button == 1){
                *state = STATE_ACCEL_CLOSE;
                *select = 0;
                *setpoint = -1;
                *button = 0;
                *feed_motor = 0;
            }
            else{
                *counter += 1;
                if(*counter > 1000){
                    *button = 1;
                    *counter = 0;
                }
            }
            break;
        
        case STATE_ACCEL_CLOSE:
            if(*feed_motor == 1){
                *state = STATE_VEL_CLOSE;
                *select = 1;
                *setpoint = -1.0;
                *feed_motor = 0;
            }
            else{
                if(*button == 1){
                    *state = STATE_ACCEL_OPEN;
                    *select = 0;
                    *setpoint = 1;
                    *feed_motor = 0;
                    *button = 0;
                }
            }
            break;
        
        case STATE_VEL_CLOSE:
            if(*feed_motor == 1){
                *state = STATE_DECEL_CLOSE;
                *select = 0;
                *setpoint = 1;
                *feed_motor = 0;

            }
            else{
                if(*button == 1){
                    *state = STATE_ACCEL_OPEN;
                    *select = 0;
                    *setpoint = 1;
                    *feed_motor = 0;
                    *button = 0;
                }
            }
            break;

        case STATE_DECEL_CLOSE:
            if(*feed_motor == 1){
                *state = STATE_OFF_CLOSE;
                *select = 1;
                *setpoint = 0.0;
                *feed_motor = 0;

            }
            else{
                if(*button == 1){
                    *state = STATE_ACCEL_OPEN;
                    *select = 0;
                    *setpoint = 1;
                    *feed_motor = 0;
                    *button = 0;
                }
            }
            break;

        default:
            break;

//        vTaskDelayUntil(&xLastWakeTime, 10/portTICK_PERIOD_MS);
        
    }
}

#endif
