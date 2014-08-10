
#include "hw_config.h"

#include "i2c.h"
#include "timer.h"
#include "rc_io.h"
#include "fram.h"

#include "param.h"

static void InitLED(void);
void loop_1hz(void);
void loop_20hz(void);
void loop_50hz(void);
void loop_100hz(void);
void loop_200hz(void);

volatile uint8_t p_flg = 0;


int32_t main(void){
	gain g = { 0.0, 0.0, 0.0};
	
	uint32_t index = 0;
	
	int32_t size;
	uint8_t data_rx[64];
	uint8_t data_tx = 0x00;
	uint32_t start, end;
	
	//‰Šú‰»ŠJŽn
	conio_init(57600UL);
	
	Init_timer();
	InitLED();
	rcin_enable(0);
	
	Init_i2c();
	Init_fram();
	Init_DT();
	
	printf("Initialize OK.\r\n");
	//‰Šú‰»I—¹
	
	i2c->Cfg.SlaveAddr = 0x42;
	i2c->Cfg.BaudRate = 400000;
	
	g.p_gain = read_float(0);
	g.i_gain = read_float(4);
	g.d_gain = read_float(8);
	
	printf("%f, %f, %f\r\n", g.p_gain, g.i_gain, g.d_gain);
	
	while(1){
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

