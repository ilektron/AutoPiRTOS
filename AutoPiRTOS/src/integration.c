/*
 * integration.c
 *
 *  Created on: Jan 18, 2012
 *      Author: ilektron
 */
#include "integration.h"

float integrate_simple(float fx, float Efx, float dt)
{
	return Efx+fx*dt;
}

float integrate_trapezoidal(float fx0, float fx1, float Efx, float dt)
{
	return Efx + 0.5f*(fx0 + fx1)*dt;
}

// Simspons integration method assuming fixed dt
float integrate_simpsons(float fx, int degree, float* fxp, float dt)
{
	return 0;
}
