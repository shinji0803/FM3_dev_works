
#include <math.h>

#include "hw_config.h"

#include "i2c.h"
#include "timer.h"
#include "rc_io.h"
#include "fram.h"

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
	vector3f gyro;
	
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
	
	uint8_t rawData[22];
	while(1){
		//px4f_update();
		//px4f_get_raw(rawData);
		gyro = AHRS_get_gyro();
		
		if(p_flg == 1){
			p_flg = 0;
			printf("%f, %f, %f \r\n", gyro.x, gyro.y, gyro.z);
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

