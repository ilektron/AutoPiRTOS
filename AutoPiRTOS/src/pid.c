/*
 * pid.c
 *
 *  Created on: Jan 11, 2012
 *      Author: ilektron
 */

#include "pid.h"

#define ABS(X)	((X)>(0)?(X):(-X))

static inline float clip(float val, float floor, float ceiling)
{
	return (val > ceiling) ? ceiling : ((val < floor) ? floor : val);
}

void pid_init(pid_t* pid, float p, float i, float d, float dt, float desired_val)
{
	pid->p = p;
	pid->i = i;
	pid->d = d;
	pid->dt = dt;
	pid->integrator = 0;
	pid->desired_val = desired_val;
}

void pid_set_gain(pid_t* pid, float p, float i, float d)
{
	pid->p = p;
	pid->i = i;
	pid->d = d;
}

void pid_zero_integrator(pid_t* pid)
{
	pid->integrator = 0;
}

/**
 * Takes an input to the system and spits out a control output.  The output
 * is between -1 and 1 and will be clipped accordingly.  The control process
 * must scale the control to the appropriate control signal for the application
 */
float pid_input(pid_t* pid, float input)
{
	float ret = 0;
	float error = pid->desired_val - input;

	// P term
	ret = pid->p*error;

	// D term - Differentiate input to avoid spikes when desired value changes
	ret -= (input - pid->last_input)*pid->d/pid->dt;

	// Clip the control at this point to prepare it for the integrator
	ret = clip(ret, -1.0f, 1.0f);

	// I term - Should not jump if I term is changed, by factoring out the I
	pid->integrator += pid->i*error*pid->dt;

	// Don't over integrate if we saturate the control
	if (ret + pid->integrator > 1.0f)
	{
		pid->integrator = 1.0f - ret;
	}
	else if (ret + pid->integrator < -1.0f)
	{
		pid->integrator = -1.0f - ret;
	}
	ret += pid->integrator;

	pid->control = ret;
	pid->last_input = input;
	return ret;
}
