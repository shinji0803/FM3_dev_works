
#include <math.h>

#include "hw_config.h"

#include "i2c.h"
#include "timer.h"
#include "rc_io.h"
#include "fram.h"

#include "LSM303DLH.h"

static void InitLED(void);
void loop_1hz(void);
void loop_20hz(void);
void loop_50hz(void);
void loop_100hz(void);
void loop_200hz(void);

volatile uint8_t p_flg = 0;


int32_t main(void){
	/*
	uint32_t index = 0;
	
	int32_t size;
	uint8_t data_rx[64];
	uint8_t data_tx = 0x00;
	uint32_t start, end;
	*/
	
	//‰Šú‰»ŠJŽn
	conio_init(57600UL);
	
	Init_timer();
	InitLED();
	rcin_enable(0);
	
	Init_i2c();
	//Init_fram();
	Init_DT();
	
	printf("Initialize OK.\r\n");
	//‰Šú‰»I—¹
	
	i2c->Cfg.BaudRate = 400000; 
	enableDefault();
	
	float accel_scale = 0.0f;
	int count = 0;
	
	while(1){
		readAcc();
		accel_scale += sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
		count ++;
			
		if(count >= 10){
			accel_scale /= 10;
			accel_scale = 9.767f / accel_scale;
			break;
		}
		
		wait(100);
	}
	
	
	while(1){
		readAcc();
		
		if(p_flg == 1){
			p_flg = 0;
			printf("%f, %f, %f, %f \r\n", accel_scale, a.x * accel_scale, a.y * accel_scale, a.z * accel_scale);
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

