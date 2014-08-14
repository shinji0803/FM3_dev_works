
#ifndef AHRS_H
#define AHRS_H

/* Attitude Heading Reference System */

#include <Math.h>

#include "myMath.h"

#include "timer.h"
#include "i2c.h"
#include "hw_config.h"
#include "PX4FLOW.h"
#include "LSM303DLH.h"

#define GRAVITY 9.767f

void AHRS_Init(void);

Vector3f AHRS_get_gyro(void);
Vector3f AHRS_get_acc(void);

#endif