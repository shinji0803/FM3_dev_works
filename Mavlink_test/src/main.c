
#include <math.h>

#include "hw_config.h"

#include "i2c.h"
#include "timer.h"
#include "rc_io.h"
#include "fram.h"

#include "myMath.h"
#include "AHRS.h"

#include "mavlink.h"
#include "mavlink_support.h"

#define VectorPrintf(v) printf("%+6.4f, %+6.4f, %+6.4f", v.x, v.y, v.z)

static void InitLED(void);
inline void loop_1hz(void);
inline void loop_20hz(void);
inline void loop_50hz(void);
inline void loop_100hz(void);
inline void loop_200hz(void);
extern volatile timeFlg time;

extern mavlink_message_t msg;

volatile uint32_t now, rev_update, delta_time;

int32_t main(void){
	
	int32_t size = 0;
	//uint32_t start, end;
	

	//‰Šú‰»ŠJŽn
	conio_init(57600UL);
	
	Init_timer();
	InitLED();
	rcin_enable(0);
	
	Init_i2c();
	Init_fram();
	Init_DT();
	
	printf("Initialize OK.\r\n");
	wait(100);
	//‰Šú‰»I—¹
	
	AHRS_Init();
	
	Mavlink_port_init(2, 57600UL);
	
	uint16_t input[8];
	uint8_t data[MAVLINK_MAX_PACKET_LEN];
	
	Vector3f gyro;
	Vector3f acc, mag;
	Vector3f attitude;
	
	while(1){
		
		Mavlink_rx_check();
		
		if(time.flg_1hz == 1){
			time.flg_1hz = 0;
			Mavlink_heartbeat_send();
		}
		
		if(time.flg_20hz == 1){
			time.flg_20hz = 0;
			rc_multiread(input);
			//Mavlink_rcin_raw_send(input);
			
			/*
			mavlink_msg_global_position_int_pack(100, 200, &msg, get_millis(), 340780080, 1345612070, 0, 0, 
												0, 0, 0, (uint16_t)(AHRS_heading((Vector3f){0,1,0}) * 100));
			size =  mavlink_msg_to_send_buffer(data, &msg);
			Mavlink_tx(data, &size);
			*/			
			
			AHRS_get_raw_gyro(&gyro);
			AHRS_get_raw_acc(&acc);
			AHRS_get_raw_mag(&mag);
			Mavlink_imu_raw_send( &acc, &gyro, &mag);
		}
		
		if(time.flg_50hz == 1){
			time.flg_50hz = 0;
			AHRS_get_euler(&attitude);
			AHRS_get_omega(&gyro);
			Mavlink_att_send(&attitude, &gyro);
			
			Mavlink_debug_send( 0, (float)(delta_time) / 1000000.f);
		}
		
		if(time.flg_100hz == 1){
			time.flg_100hz = 0;
			
			AHRS_read_imu();
			
			now = get_micros();
			delta_time = now - rev_update;
			AHRS_dcm_update((float)delta_time / 1000000.f);
			//AHRS_dcm_update(0.01f);
			rev_update = now;
			
			AHRS_dcm_normalize();
			AHRS_drift_correction();
			
		}
	} 
}


static void InitLED(){
	FM3_GPIO->PFRF_f.P3		= 0; 
	FM3_GPIO->PZRF_f.P3		= 1; 
	FM3_GPIO->DDRF_f.P3		= 1;
	FM3_GPIO->PDORF_f.P3	= 0;
}

inline void loop_1hz(){
	if(time.calibrate != 1) FM3_GPIO->PDORF_f.P3 = ~FM3_GPIO->PDORF_f.P3;
}

inline void loop_20hz(){
	if(time.calibrate != 0) FM3_GPIO->PDORF_f.P3 = ~FM3_GPIO->PDORF_f.P3;
}

inline void loop_50hz(){

}

inline void loop_100hz(){
	/*
	now = get_micros();
	delta_time = now - rev_update;
	//AHRS_dcm_update((float)(now - rev_update) / 1000000.f);
	AHRS_dcm_update(0.01f);
	rev_update = now;
	
	AHRS_dcm_normalize();
	AHRS_drift_correction();
	*/
}

inline void loop_200hz(){

}

