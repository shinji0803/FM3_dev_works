
#include "AHRS.h"

static Matrix3f dcm_matrix;

static float accScale;
static Vector3f gyroOffset;

static void init_rotation_matrix(Matrix3f *m, float yaw, float pitch, float roll);

void AHRS_Init(){
	Vector3f temp = { 0.0f, 0.0f, 0.0f};
	Vector3f temp1, temp2;
	Vector3f sum = { 0.0f, 0.0f, 0.0f};
	
	const Vector3f xAxis = {1.0f, 0.0f, 0.0f};
	
	int32_t sum_count = 0;

	/* Acc and Mag Initialize*/
	LSM303DLH_Init();
	
	wait(100);
	
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
	sum_count = 0;
	while(1){
		px4f_get_gyro(&temp);
		sum.x += temp.x;
		sum.y += temp.y;
		sum.z += temp.z;
		sum_count ++;
		
		cputs(".");
		wait(200);
		if(sum_count >= 10) break;
	}
	printf("\r\n");
	gyroOffset.x = sum.x / 10.0f; 
	gyroOffset.y = sum.y / 10.0f; 
	gyroOffset.z = sum.z / 10.0f;
	
	//Initialize DCM Matrix
	Vector3f att;
	
	//Get Roll and Pitch 
	readAcc(&temp);
	att.y = -atan2(temp.x, sqrt(temp.y * temp.y + temp.z * temp.z));
	Vector_Cross_Product(&temp1, &temp, &xAxis);
	Vector_Cross_Product(&temp2, &xAxis, &temp1);
	att.x = -atan2( temp2.y, temp2.z);
	//Get Yaw
	att.z = heading((Vector3f){0,-1,0});
	init_rotation_matrix(&dcm_matrix, att.z, att.y, att.x);
}

void AHRS_get_gyro(Vector3f *g){
	
	px4f_get_gyro(g);
	g->x = g->x - gyroOffset.x;
	g->y = g->y - gyroOffset.y;
	g->z = g->z - gyroOffset.z;
	
}

void AHRS_get_acc(Vector3f *a){

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
