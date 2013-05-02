/*! 
 *  Implementation of the quaternion concept. 
 *
 *  Used to update the attitude without suffering gimbal lock.
 *  For more information, please see http://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
 *
 *  @file     quaternion.c
 *  @author   Tom Pycke
 *  @date     25-oct-2008
 *  @since    0.1
 */
 
#include <errno.h>
#include <math.h>
#include "quaternion.h"

/*!
 *	 Initialize quaternion with roll, pitch and yaw euclidian angles.
 */
void quaternion_from_attitude(const float roll, const float pitch, const float yaw, quaternion* q)
{
	float cos_roll_2 = cosf(roll/2.0);
	float sin_roll_2 = sinf(roll/2.0);
	float cos_pitch_2 = cosf(pitch/2.0);
	float sin_pitch_2 = sinf(pitch/2.0);
	float cos_yaw_2 = cosf(yaw/2.0);
	float sin_yaw_2 = sinf(yaw/2.0);

	q->e0 = cos_roll_2 * cos_pitch_2 * cos_yaw_2 + sin_roll_2 * sin_pitch_2 * sin_yaw_2;
	q->e1 = sin_roll_2 * cos_pitch_2 * cos_yaw_2 - cos_roll_2 * sin_pitch_2 * sin_yaw_2;
	q->e2 = cos_roll_2 * sin_pitch_2 * cos_yaw_2 + sin_roll_2 * cos_pitch_2 * sin_yaw_2;
	q->e3 = cos_roll_2 * cos_pitch_2 * sin_yaw_2 - sin_roll_2 * sin_pitch_2 * cos_yaw_2; // WAS cos_roll_2 * cos_pitch_2 * sin_yaw_2 - sin_roll_2 * sin_pitch_2 * sin_yaw_2

	quaternion_normalize(q);
}


/*!
 *	 Update quaternion with rates.
 *   @param p = along the x-axis = roll-rate
 *   @param q = along the y-axis = pitch-rate
 *   @param z = along the z-axis = yaw-rate
 */
void quaternion_update_with_rates(const float rollrate, const float pitchrate, const float headingrate, quaternion* q, const float dt)
{
	const float w1 = rollrate;
	const float w2 = pitchrate;
	const float w3 = headingrate;

	float q0 = q->e0;
	float q1 = q->e1;
	float q2 = q->e2;
	float q3 = q->e3;
	
	q0 += 0.5 * (         - q->e1*w1 - q->e2*w2 - q->e3*w3)*dt;
	q1 += 0.5 * (q->e0*w1 +            q->e2*w3 - q->e3*w2)*dt;
	q2 += 0.5 * (q->e0*w2 - q->e1*w3 +            q->e3*w1)*dt;
	q3 += 0.5 * (q->e0*w3 + q->e1*w2 - q->e2*w1           )*dt;
	
	q->e0 = q0;
	q->e1 = q1;
	q->e2 = q2;
	q->e3 = q3;

	quaternion_normalize(q);
}

float quaternion_to_roll(const quaternion* q)
{
	float ret;
	errno = 0;
	ret = atan2( 2.0f * (q->e2*q->e3 + q->e0*q->e1) ,
	             (1.0f - 2.0f * (q->e1*q->e1 + q->e2*q->e2)));
	if (errno)
	{
		return 0.0f;
	}
	else
	{
		return ret;
	}
}

float quaternion_to_pitch(const quaternion* q)
{
	float ret;
	errno = 0;
	ret = asinf( -2.0 * (q->e1*q->e3 - q->e0*q->e2) );
	if (errno)
	{
		return 0.0f;
	}
	else
	{
		return ret;
	}
}


float quaternion_to_yaw(const quaternion* q)
{
	float ret;
	errno = 0;
	ret = atan2( 2.0f * (q->e0*q->e3 + q->e1*q->e2),
	          (1.0f - 2.0f * (q->e2*q->e2 + q->e3*q->e3)));

	if (errno)
	{
		return 0.0f;
	}
	else
	{
		return ret;
	}
}

#define NORM_ERROR		0.0001f
void quaternion_normalize(quaternion* q)
{
	float norm = q->e0*q->e0
			+ q->e1*q->e1
			+ q->e2*q->e2
			+ q->e3*q->e3;

	if (norm > (1.0f + NORM_ERROR) || norm < (1.0f - NORM_ERROR))
	{
		norm = sqrtf(norm);
		q->e0 /= norm;
		q->e1 /= norm;
		q->e2 /= norm;
		q->e3 /= norm;
	}
}

