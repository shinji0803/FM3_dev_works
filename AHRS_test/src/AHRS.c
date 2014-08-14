
#include "AHRS.h"

static float dcm_matrix[3][3];

static float accScale;
static vector3f gyroOffset;

void AHRS_Init(){
	vector3f temp = { 0.0f, 0.0f, 0.0f};
	vector3f sum = { 0.0f, 0.0f, 0.0f};
	int32_t sum_count = 0;

	/* Acc and Mag Initialize*/
	LSM303DLH_Init();
	
	wait(100);
	
	/* Acc scale calibration */
	cputs("Acc Calibration");
	while(1){
		temp = readAcc();
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
		temp = px4f_get_gyro();
		sum.x += temp.x;
		sum.y += temp.y;
		sum.z += temp.z;
		sum_count ++;
		
		cputs(".");
		wait(100);
		if(sum_count >= 10) break;
	}
	printf("\r\n");
	gyroOffset.x = sum.x / 10.0f; 
	gyroOffset.y = sum.y / 10.0f; 
	gyroOffset.z = sum.z / 10.0f; 
}

vector3f AHRS_get_gyro(){
	vector3f temp;
	
	temp = px4f_get_gyro();
	temp.x = temp.x - gyroOffset.x;
	temp.y = temp.y - gyroOffset.y;
	temp.z = temp.z - gyroOffset.z;
	
	return temp;
}