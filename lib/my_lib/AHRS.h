
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
#include "WMP.h"

#define GRAVITY 9.767f

#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define RAD_TO_DEG 57.295779513082320876798154814105f

#define ToRad(x) (x * DEG_TO_RAD)
#define ToDeg(x) (x * RAD_TO_DEG)

void AHRS_Init(void);

void AHRS_get_gyro(Vector3f *g);
void AHRS_get_raw_gyro(Vector3f *g);

void AHRS_get_acc(Vector3f *a);

void AHRS_get_euler(Vector3f *att);

#endif