
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


int32_t main(void){
	
	int32_t size = 0;
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
	wait(100);
	//‰Šú‰»I—¹
	
	AHRS_Init();
	
	Mavlink_port_init(2, 57600UL);
	
	uint16_t input[8];
	int system_type = MAV_TYPE_QUADROTOR;
	int autopilot_type = MAV_AUTOPILOT_GENERIC;
	mavlink_message_t msg;
	uint8_t data[MAVLINK_MAX_PACKET_LEN];
	
	Vector3f gyro;
	Vector3f acc, mag;
	
	while(1){
		
		Mavlink_rx_check();
		
		if(time.flg_1hz == 1){
			time.flg_1hz = 0;
			mavlink_msg_heartbeat_pack(100, 200, &msg, system_type, autopilot_type, 0, 0, 0);
			
			size = mavlink_msg_to_send_buffer(data, &msg);
			Mavlink_tx(data, &size);
		}
		
		if(time.flg_20hz == 1){
			time.flg_20hz = 0;
			rc_multiread(input);
			mavlink_msg_rc_channels_raw_pack(100, 200, &msg, get_millis(), 0 , input[0], input[1], input[2],
												input[3], input[4], input[5], input[6], input[7],  255);
			
			size =  mavlink_msg_to_send_buffer(data, &msg);
			Mavlink_tx(data, &size);
			
			mavlink_msg_global_position_int_pack(100, 200, &msg, get_millis(), 340780080, 1345612070, 0, 0, 
												0, 0, 0, (uint16_t)(heading((Vector3f){0,1,0}) * 100));
			size =  mavlink_msg_to_send_buffer(data, &msg);
			Mavlink_tx(data, &size);
			
		}
		
		if(time.flg_50hz == 1){
			time.flg_50hz = 0;
			start = get_micros();
			AHRS_get_raw_gyro(&gyro);
			readAcc(&acc);
			readMag(&mag);
			end = get_micros();
			
			mavlink_msg_raw_imu_pack(100, 200, &msg, get_micros(), (int16_t)acc.x, (int16_t)acc.y, (int16_t)acc.z,
									gyro.x, gyro.y, gyro.z, (int16_t)mag.x, (int16_t)mag.y, (int16_t)mag.z);
			
			size = mavlink_msg_to_send_buffer(data, &msg);
			Mavlink_tx(data, &size);
			/*
			mavlink_msg_debug_pack(100, 200, &msg, get_millis(), 0, (float)(end-start));
			size = mavlink_msg_to_send_buffer(data, &msg);
			Mavlink_tx(data, &size);
			*/
			
			AHRS_get_gyro(&gyro);
			uint8_t name[] = "Gyro";
			mavlink_msg_debug_vect_pack(100, 200, &msg, name, get_micros(), gyro.x, gyro.y, gyro.z);
			size = mavlink_msg_to_send_buffer(data, &msg);
			Mavlink_tx(data, &size);
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
	FM3_GPIO->PDORF_f.P3 = ~FM3_GPIO->PDORF_f.P3;
}

inline void loop_20hz(){

}

inline void loop_50hz(){

}

inline void loop_100hz(){

}

inline void loop_200hz(){

}

