
#include "AHRS.h"

static Matrix3f dcm_matrix;

static float accScale;
static Vector3f gyroOffset;

static void init_rotation_matrix(Matrix3f *m, float yaw, float pitch, float roll);
static uint8_t check_gyro_calib_data(Vector3f *g);


void AHRS_Init(void)
{
	Vector3f temp = { 0.0f, 0.0f, 0.0f};
	Vector3f temp1, temp2;
	Vector3f sum = { 0.0f, 0.0f, 0.0f};
	
	const Vector3f xAxis = {1.0f, 0.0f, 0.0f};
	
	int32_t sum_count = 0;

	/* Acc and Mag enable */
	LSM303DLH_Init();
	/* Gyro enable */
	WMP_init();
	
	wait(1000);
	
	/* Acc scale calibration */
	cputs("Acc Calibration");
	while(1){
		readAcc(&temp);
		accScale += sqrt(temp.x * temp.x + temp.y * temp.y + temp.z * temp.z);
		sum_count ++;
		
		cputs(".");
		wait(100);
		if(sum_count >= 5) break;
	}
	printf("\r\n");
	accScale = GRAVITY / (accScale / 5.0f);
	
	/* Gyro scale calibration */
	cputs("Gyro Calibration");
	
	while(1){
		sum_count = 0;
		while(1){
#ifdef USE_PX4F_GYRO		
			px4f_get_gyro(&temp); // Gyro get from PX4FLOW
#else
			WMP_get_raw_gyro(&temp); // Gyro get from WiiMotion+
#endif			
			if(check_gyro_calib_data(&temp) != 0) break;
			sum.x += temp.x;
			sum.y += temp.y;
			sum.z += temp.z;
			sum_count ++;
			
			cputs(".");
			wait(200);
			if(sum_count >= 10) break;
		}
		
		if(sum_count >= 10) break;
		else{
			cputs("\r\nDo not move IMU.\r\nRecalibrate");
			sum.x = sum.y = sum.z = 0.0f;
			sum_count = 0;
			wait(500);
		}
		
	}
	printf("\r\n");
	gyroOffset.x = sum.x / 10.0f; 
	gyroOffset.y = sum.y / 10.0f; 
	gyroOffset.z = sum.z / 10.0f;
	
	printf("Gyro Offset ( %f, %f, %f)\r\n", gyroOffset.x, gyroOffset.y, gyroOffset.z);
	//Initialize DCM Matrix
	Vector3f att;
	
	//Get Roll and Pitch 
	readAcc(&temp);
	att.y = -atan2(temp.x, sqrt(temp.y * temp.y + temp.z * temp.z));
	Vector_Cross_Product(&temp1, &temp, &xAxis);
	Vector_Cross_Product(&temp2, &xAxis, &temp1);
	att.x = -atan2( temp2.y, temp2.z);
	//Get Yaw
	att.z = heading((Vector3f){0,1,0});
	init_rotation_matrix(&dcm_matrix, att.z, att.y, att.x);
}

#define GYRO_CALIB_LOW	7800
#define GYRO_CALIB_HIGH 8400
static uint8_t check_gyro_calib_data(Vector3f *g)
{
	uint8_t error_count = 0;
	
	if(g->x > GYRO_CALIB_HIGH || g->x < GYRO_CALIB_LOW) error_count ++;
	if(g->y > GYRO_CALIB_HIGH || g->y < GYRO_CALIB_LOW) error_count ++;
	if(g->z > GYRO_CALIB_HIGH || g->z < GYRO_CALIB_LOW) error_count ++;
	return error_count;
}

#define GYRO_SCALE 13.768f
void AHRS_get_gyro(Vector3f *g)
{
	AHRS_get_raw_gyro(g);
	
	g->x = ToRad(g->x / GYRO_SCALE);
	g->y = ToRad(g->y / GYRO_SCALE);
	g->z = ToRad(g->z / GYRO_SCALE);
}

void AHRS_get_raw_gyro(Vector3f *g)
{
	uint8_t slow_mode;
	
#ifdef USE_PX4F_GYRO	
	px4f_get_gyro(g);
#else
	WMP_get_raw_gyro(g);
	WMP_get_gyro_mode(&slow_mode);
#endif
	
	g->x = g->x - gyroOffset.x;
	g->y = -(g->y - gyroOffset.y); // Pitch reversed
	g->z = g->z - gyroOffset.z;
	
	if(((slow_mode >> 2) & 0x01) != 1) g->x *= 4.545454f; // When high speed mode, multiply value by (2000/440)
	if(((slow_mode >> 1) & 0x01) != 1) g->y *= 4.545454f;
	if(((slow_mode >> 0) & 0x01) != 1) g->z *= 4.545454f; 
	
}

void AHRS_get_acc(Vector3f *a)
{
	readAcc(a);
	a->x = a->x * accScale;
	a->y = a->y * accScale;
	a->z = a->z * accScale;
}


void init_rotation_matrix(Matrix3f *m, float yaw, float pitch, float roll)
{
  float c1 = cos(roll);
  float s1 = sin(roll);
  float c2 = cos(pitch);
  float s2 = sin(pitch);
  float c3 = cos(yaw);
  float s3 = sin(yaw);

  // Euler angles, right-handed, intrinsic, XYZ convention
  // (which means: rotate around body axes Z, Y', X'') 
  m->a.x = c2 * c3;
  m->a.y = c3 * s1 * s2 - c1 * s3;
  m->a.z = s1 * s3 + c1 * c3 * s2;

  m->b.x = c2 * s3;
  m->b.y = c1 * c3 + s1 * s2 * s3;
  m->b.z = c1 * s2 * s3 - c3 * s1;

  m->c.x = -s2;
  m->c.y = c2 * s1;
  m->c.z = c1 * c2;
}

void AHRS_get_euler(Vector3f *att)
{
	att->x = -asin(dcm_matrix.c.x);
	att->y = atan2(dcm_matrix.c.y, dcm_matrix.c.z);
	att->z = atan2(dcm_matrix.b.x, dcm_matrix.a.x);
}
	
	
