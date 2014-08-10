
#include "hw_config.h"

#include "i2c.h"
#include "timer.h"
#include "rc_io.h"
#include "fram.h"

static void InitLED(void);
void loop_1hz(void);
void loop_20hz(void);
void loop_50hz(void);
void loop_100hz(void);
void loop_200hz(void);

volatile uint8_t p_flg = 0;


int32_t main(void){
	
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
	//Init_fram();
	Init_DT();
	
	printf("Initialize OK.\r\n");
	//‰Šú‰»I—¹
	
	i2c->Cfg.SlaveAddr = 0x42;
	i2c->Cfg.BaudRate = 400000;
	
	
	while(1){
		start = get_micros();
		size = 1;
		i2c->DataTx(&data_tx, &size);
		size = 22;
		i2c->DataRx(data_rx, &size);
		end = get_micros();
		
		if(p_flg == 1){
			p_flg = 0;
			printf("%ld\t", (end - start));
			for(index = 0; index < 22; index ++) printf("%2x ", data_rx[index]);
			printf("\r\n");
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

