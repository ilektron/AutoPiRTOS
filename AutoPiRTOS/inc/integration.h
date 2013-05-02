/*
 * integration.h
 *
 *  Created on: Jan 18, 2012
 *      Author: ilektron
 */

#ifndef INTEGRATION_H_
#define INTEGRATION_H_


float integrate_simple(float fx, float fxp, float dt);
float integrate_trapezoidal(float fx0, float fx1, float Efx, float dt);
float integrate_simpsons(float fx, int degree, float* fxp, float dt);

#endif /* INTEGRATION_H_ */
