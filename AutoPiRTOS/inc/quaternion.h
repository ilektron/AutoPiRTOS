#ifndef QUATERNION_H
#define QUATERNION_H

typedef struct QUAT
{
	float e0;
	float e1;
	float e2;
	float e3;
} quaternion;


void quaternion_from_attitude(const float pitch, const float roll, const float yaw, quaternion* q);

void quaternion_update_with_rates(const float rollrate, const float pitchrate, const float headingrate, quaternion* q, const float dt);

float quaternion_to_roll(const quaternion* q);

float quaternion_to_pitch(const quaternion* q);

float quaternion_to_yaw(const quaternion* q);

void quaternion_normalize(quaternion* q);

#endif
