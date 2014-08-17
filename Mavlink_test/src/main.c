
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

/* Mavlink Output Setting */
#include "UartDev.h"
#include "checksum.h"
UartDev_IOB *mavlink;
uint8_t packetCount = 0;



int32_t main(void){
	
	int32_t size = 0;
	uint8_t data[64];
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
	wait(100);
	//‰Šú‰»I—¹
	
	/* Mavlink Output Port Setting */
	FM3_GPIO->PFR7 = FM3_GPIO->PFR7 | 0x0C;
	FM3_GPIO->EPFR07 = FM3_GPIO->EPFR07 | 0x40000;
	mavlink = &UartDev[2];
	mavlink->Cfg.BufSize = 256;
	mavlink->Cfg.BaudRate = 57600;
	mavlink->Cfg.BitOrder = UartDev_BITORDER_LSB;
	mavlink->Init();
	
	uint16_t input[8];
	uint32_t time_boot_ms;
	/* Setting End */
		
	while(1){
		//start = get_micros();
		//end = get_micros();
		rc_multiread(input);
		
		if(p_flg == 1){
			p_flg = 0;
			memset(data, '\0', sizeof(data));
			
			// Mavlink Heartbeat Message
			data[0] = 0xFE; //Start sign = 0xFE
			data[1] = 22; //Payload length 
			data[2] = packetCount; //Packet Sequence
			data[3] = 0x01; //System ID
			data[4] = 0x01; //Component ID
			data[5] = 35; //Message ID
			
			time_boot_ms = get_millis();
			for(int i = 0; i < 4; i++){
				data[6 + i] = (time_boot_ms >> (i*8)) & 0xFF;
			}
			
			for(int i = 0; i < 8; i++){
				data[10 + (2*i)] = input[i] & 0xFF;
				data[10 + (2*i+1)] = (input[i] >> 8) & 0xFF;
			}
			data[26] = 0;
			data[27] = 0;
			
			uint16_t checksum = crc_calculate((const uint8_t*)&data[1], 5);
			//crc_accumulate_buffer(&checksum, &data[6], 22);
			//crc_accumulate(crc_extra, &checksum);
			
			data[28] = checksum & 0xFF;
			data[29] = checksum >> 8;
			packetCount ++;
			
			
			size = strlen(data);
			mavlink->BufTx(data, &size, UartDev_FLAG_BLOCKING);
			
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

