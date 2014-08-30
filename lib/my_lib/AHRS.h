
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

extern volatile timeFlg time;

void AHRS_Init(void);

void AHRS_read_imu(void);

void AHRS_dcm_update(float dt);
void AHRS_dcm_normalize(void);
void AHRS_drift_correction(void);

void AHRS_get_gyro(Vector3f *g);
void AHRS_get_raw_gyro(Vector3f *g);

void AHRS_get_acc(Vector3f *a);
void AHRS_get_raw_acc(Vector3f *a);

void AHRS_get_mag(Vector3f *m);
void AHRS_get_raw_mag(Vector3f *m);

void AHRS_get_euler(Vector3f *att);

float AHRS_heading(Vector3f from);

// DCM parameters
#define Kp_ROLLPITCH 0.02f
#define Ki_ROLLPITCH 0.00002f
#define Kp_YAW 1.2f
#define Ki_YAW 0.00002f

/* Calibration Data */
/* Mag calibration data */
static const Vector3f m_max = { 400.0f, 400.0f, 400.0f}; 
static const Vector3f m_min = { -400.0f, -400.0f, -400.0f};

/* Acc calibration data */
static const Vector3f a_max = { 16500.0f, 15800.0f, 16800.0f};
static const Vector3f a_min = { -16550.0f, -16500.0f, -16100.0f};

/* Gyro calibration data */
/* Gyro valid range at rest when use WiiMotionPlus */
#define GYRO_CALIB_LOW	7800
#define GYRO_CALIB_HIGH 8400


/* Sensor scale data */
#define GYRO_SCALE 13.768f // 13.768 unit/(deg/sec) when use WiiMotionPlus

#define ACCEL_X_OFFSET ((a_min.x + a_max.x) / 2.0f)
#define ACCEL_Y_OFFSET ((a_min.y + a_max.y) / 2.0f)
#define ACCEL_Z_OFFSET ((a_min.z + a_max.z) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (a_max.x - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (a_max.y - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (a_max.z - ACCEL_Z_OFFSET))


#endif