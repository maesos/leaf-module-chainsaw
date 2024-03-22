/*
 * utils_generic.h
 *
 *  Created on: Dec. 24, 2021
 *      Author: Someone
 */

#ifndef UTILS_GENERIC_H_
#define UTILS_GENERIC_H_


#include <string.h>
#include <stdio.h>
#include <stdint.h>



#define ug_pid_int int64_t

typedef struct ug_pid_s {

    ug_pid_int kp;
    ug_pid_int ki;
    ug_pid_int kd;

    ug_pid_int err_now;
    ug_pid_int err_prv;
    ug_pid_int err_sum;

    ug_pid_int target;

    ug_pid_int loops;

    uint8_t use_endstop;
    ug_pid_int endstop_h;
    ug_pid_int endstop_l;

    uint8_t use_i_lim;
    ug_pid_int i_lim_h;
    ug_pid_int i_lim_l;

    uint8_t use_bias;
    ug_pid_int bias;

    uint8_t exponential;

} ug_pid_t;

int ug_pid_init( ug_pid_t * pid , ug_pid_int kp, ug_pid_int ki, ug_pid_int kd, ug_pid_int target );
int ug_pid_update( ug_pid_t * pid , ug_pid_int measured, ug_pid_int * command);
int ug_pid_set_target( ug_pid_t * pid , ug_pid_int target );
int ug_pid_set_endstop( ug_pid_t * pid , ug_pid_int end_l , ug_pid_int end_h );
int ug_pid_set_i_lim( ug_pid_t * pid , ug_pid_int i_lim_l , ug_pid_int i_lim_h );
int ug_pid_set_bias( ug_pid_t * pid , ug_pid_int target );
int ug_pid_set_exponential( ug_pid_t * pid , ug_pid_int exponential );



#endif /* UTILS_GENERIC_H_ */
