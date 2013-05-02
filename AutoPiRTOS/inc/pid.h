/*
 * pid.h
 *
 *  Created on: Jan 11, 2012
 *      Author: ilektron
 */

#ifndef PID_H_
#define PID_H_

typedef struct PID
{
	float p;
	float i;
	float d;
	float dt;
	float control;
	float last_input;
	float desired_val;
	float integrator;
} pid_t;

void pid_init(pid_t* pid, float p, float i, float d, float dt, float desired_val);
void pid_set_gain(pid_t* pid, float p, float i, float d);
void pid_zero_integrator(pid_t* pid);
float pid_input(pid_t* pid, float input);

#endif /* PID_H_ */
