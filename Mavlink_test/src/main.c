
#include <math.h>

#include "hw_config.h"

#include "i2c.h"
#include "timer.h"
#include "rc_io.h"
#include "fram.h"

#include "myMath.h"
#include "PX4FLOW.h"
#include "AHRS.h"

#include "mavlink.h"

#define VectorPrintf(v) printf("%+6.4f, %+6.4f, %+6.4f", v.x, v.y, v.z)

static void InitLED(void);
void loop_1hz(void);
void loop_20hz(void);
void loop_50hz(void);
void loop_100hz(void);
void loop_200hz(void);


/* Mavlink Output Setting */
#include "UartDev.h"
UartDev_IOB *mavlink;
uint8_t packetCount = 0;

typedef struct timeFlg{
	unsigned int flg_1hz   :	1;
	unsigned int flg_10hz  :	1;
	unsigned int flg_20hz  :	1;
	unsigned int flg_50hz  :	1;
	unsigned int flg_100hz :	1;
	unsigned int flg_200hz :	1;
	unsigned int reserve1  :	1;
	unsigned int reserve2  :	1;
} timeFlg;

volatile timeFlg time = { 0, 0, 0, 0, 0, 0, 0, 0};

int32_t main(void){
	
	int32_t size = 0;

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
	
	int system_type = MAV_TYPE_QUADROTOR;
	int autopilot_type = MAV_AUTOPILOT_GENERIC;
	mavlink_message_t msg;
	uint8_t data[MAVLINK_MAX_PACKET_LEN];
	uint8_t receive[64];
	/* Setting End */
	
	while(1){
		
		if(mavlink->RxAvailable() > 0){
			size = 1;
			mavlink->BufRx(receive, &size, UartDev_FLAG_NONBLOCKING);
			
			if(size == 1){
				if(receive[0] == MAVLINK_STX) printf("\r\n");
				printf("%2x ", receive[0]);
			}
		}
	
		if(time.flg_1hz == 1){
			time.flg_1hz = 0;
			mavlink_msg_heartbeat_pack(100, 200, &msg, system_type, autopilot_type, 0, 0, 0);
			
			size = mavlink_msg_to_send_buffer(data, &msg);
			//printf("%ld\r\n", size);
			mavlink->BufTx(data, &size, UartDev_FLAG_BLOCKING);
		}
		
		if(time.flg_20hz == 1){
			time.flg_20hz = 0;
			rc_multiread(input);
			mavlink_msg_rc_channels_raw_pack(100, 200, &msg, get_millis(), 0 , input[0], input[1], input[2],
												input[3], input[4], input[5], input[6], input[7],  0);
			
			size =  mavlink_msg_to_send_buffer(data, &msg);
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
	time.flg_1hz = 1;
	
	FM3_GPIO->PDORF_f.P3 = ~FM3_GPIO->PDORF_f.P3;
}

void loop_20hz(){
	time.flg_20hz = 1;
}

void loop_50hz(){
	time.flg_50hz = 1;
}

void loop_100hz(){
	time.flg_100hz = 1;
}

void loop_200hz(){
	time.flg_200hz = 1;
}

