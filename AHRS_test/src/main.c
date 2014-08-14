
#include <math.h>

#include "hw_config.h"

#include "i2c.h"
#include "timer.h"
#include "rc_io.h"
#include "fram.h"

#include "myMath.h"
#include "PX4FLOW.h"
#include "AHRS.h"

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
	
	//èâä˙âªäJén
	conio_init(57600UL);
	
	Init_timer();
	InitLED();
	rcin_enable(0);
	
	Init_i2c();
	//Init_fram();
	Init_DT();
	
	printf("Initialize OK.\r\n");
	wait(100);
	//èâä˙âªèIóπ
	
	px4f_init(&fData);
	AHRS_Init();
		
	while(1){
		start = get_micros();
		gyro = AHRS_get_gyro();
		acc = AHRS_get_acc();
		end = get_micros();
		
		if(p_flg == 1){
			p_flg = 0;
			printf("%4ld ", (end - start));
			printf("%+6.4f, %+6.4f, %+6.4f ", gyro.x, gyro.y, gyro.z);
			printf("%+6.4f, %+6.4f, %+6.4f \r\n", acc.x, acc.y, acc.z);
			//for(int i = 0; i < 22; i ++) printf("%x ", rawData[i]);
			//printf("\r\n");
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

