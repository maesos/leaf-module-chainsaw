/*
 * utils_generic.c
 *
 *  Created on: Dec. 24, 2021
 *      Author: Someone
 */

#include "utils_generic.h"


int ug_pid_init( ug_pid_t * pid , ug_pid_int kp, ug_pid_int ki, ug_pid_int kd, ug_pid_int target )
{

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->target = target;

    pid->use_endstop = 0;
    pid->use_bias = 0;

    pid->loops = 0;
    pid->err_sum = 0;

    pid->exponential = 0;

    return 0;
}

int ug_pid_set_target( ug_pid_t * pid , ug_pid_int target )
{

//    pid->target = target << pid->exponential;
    pid->target = target;

    return 0;

}

int ug_pid_set_endstop( ug_pid_t * pid , ug_pid_int end_l , ug_pid_int end_h )
{

    if (end_h == end_l)
    {
        pid->use_endstop = 0;
    }
    else
    {
        pid->use_endstop = 1;
        pid->endstop_h = end_h > end_l ? end_h : end_l;
        pid->endstop_l = end_l < end_h ? end_l : end_h;
//        pid->endstop_h = pid->endstop_h << pid->exponential;
//        pid->endstop_l = pid->endstop_l << pid->exponential;
    }

    return 0;
}

int ug_pid_set_i_lim( ug_pid_t * pid , ug_pid_int i_lim_l , ug_pid_int i_lim_h )
{

    if (i_lim_l == i_lim_h)
    {
        pid->use_i_lim = 0;
    }
    else
    {
        pid->use_i_lim = 1;
        pid->i_lim_h = i_lim_h > i_lim_l ? i_lim_h : i_lim_l;
        pid->i_lim_l = i_lim_l < i_lim_h ? i_lim_l : i_lim_h;
//        pid->i_lim_h = pid->i_lim_h << pid->exponential;
//        pid->i_lim_l = pid->i_lim_l << pid->exponential;
    }

    return 0;
}


int ug_pid_set_bias( ug_pid_t * pid , ug_pid_int bias )
{

    if (bias == 0)
    {
        pid->use_bias = 0;
    }
    else
    {
        pid->use_bias = 1;
        pid->bias = bias;
    }

    return 0;
}

int ug_pid_update( ug_pid_t * pid , ug_pid_int measured, ug_pid_int * command)
{

//    measured = measured << pid->exponential;

    pid->err_prv = pid->loops++ > 0 ? pid->err_now : 0;
//    pid->err_now = measured - pid->target;
    pid->err_now = pid->target - measured;
    pid->err_sum += pid->err_now;
//    pid->err_sum = pid->err_sum;

    if (pid->use_i_lim != 0)
    {
        pid->err_sum = pid->err_sum > pid->i_lim_h ? pid->i_lim_h : pid->err_sum;
        pid->err_sum = pid->err_sum < pid->i_lim_l ? pid->i_lim_l : pid->err_sum;
    }
    
    *command = pid->kp * pid->err_now;
    *command += pid->ki * pid->err_sum;
    *command += pid->kd != 0 ? pid->kd * (pid->err_now - pid->err_prv) : 0 ;

    if (pid->use_bias != 0)
    {
        *command += pid->bias;
    }

    if (pid->use_endstop != 0)
    {
        *command = *command > pid->endstop_h ? pid->endstop_h : *command;
        *command = *command < pid->endstop_l ? pid->endstop_l : *command;
    }

    *command = *command >> pid->exponential;

    return 0;
}

int ug_pid_set_exponential( ug_pid_t * pid , ug_pid_int exponential )
{

    pid->exponential = exponential;

    return 0;

}


