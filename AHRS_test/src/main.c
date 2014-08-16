
#include <math.h>

#include "hw_config.h"

#include "i2c.h"
#include "timer.h"
#include "rc_io.h"
#include "fram.h"

#include "myMath.h"
#include "PX4FLOW.h"
#include "AHRS.h"

#define VectorPrintf(v) printf("%+6.4f, %+6.4f, %+6.4f", v.x, v.y, v.z)

static void InitLED(void);
void loop_1hz(void);
void loop_20hz(void);
void loop_50hz(void);
void loop_100hz(void);
void loop_200hz(void);

volatile uint8_t p_flg = 0;


int32_t main(void){
	
	uint32_t start, end;
	
	flow_data fData;
	Vector3f gyro, acc;
	Vector3f temp1, temp2;
	Vector3f xAxis = {-1.0f, 0.0f, 0.0f};
	Vector3f att = { 0.0f, 0.0f, 0.0f};
	
	//‰Šú‰»ŠJŽn
	conio_init(57600UL);
	
	Init_timer();
	InitLED();
	rcin_enable(0);
	
	Init_i2c();
	//Init_fram();
	Init_DT();
	
	printf("Initialize OK.\r\n");
	wait(100);
	//‰Šú‰»I—¹
	
	px4f_init(&fData);
	AHRS_Init();
		
	while(1){
		start = get_micros();
		//AHRS_get_gyro(&gyro);
		//AHRS_get_acc(&acc);
		readAcc(&acc);
		end = get_micros();
		
		if(p_flg == 1){
			att.y = -atan2(acc.x, sqrt(acc.y * acc.y + acc.z * acc.z));
			Vector_Cross_Product(&temp1, &acc, &xAxis);
			Vector_Cross_Product(&temp2, &xAxis, &temp1);
			att.x = atan2( temp2.y, temp2.z);
			
			VectorPrintf(att);
			printf("\r\n");
			p_flg = 0;
			
		}
	} 
}


static void InitLED(){
	FM3_GPIO->PFRF_f.P3		= 0; 
	FM3_GPIO->PZRF_f.P3		= 1; 
	FM3_GPIO->DDRF_f.P3		= 1;
	FM3_GPIO->PDORF_f.P3	= 0;
}

void loop_1hz(){
	FM3_GPIO->PDORF_f.P3 = ~FM3_GPIO->PDORF_f.P3;	
}

void loop_20hz(){
	p_flg = 1;
}

void loop_50hz(){

}

void loop_100hz(){
	
}

void loop_200hz(){
	
}

