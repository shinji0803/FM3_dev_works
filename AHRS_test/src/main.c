
#include <math.h>

#include "hw_config.h"

#include "i2c.h"
#include "timer.h"
#include "rc_io.h"
#include "fram.h"

#include "PX4FLOW.h"
#include "LSM303DLH.h"

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
	px4f_init(&fData);
	
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
	
	m_max.x = 0.0f; m_max.y = 0.0f; m_max.z = 0.0f;
	m_min.x = 0.0f; m_min.y = 0.0f; m_min.z = 0.0f;
	
	while(1){
		start = get_micros();
		readAcc();
		readMag();
		
		px4f_update();
		calc_flow();
		end = get_micros();
		
		if(p_flg == 1){
			p_flg = 0;
			printf("%ld, ", (end - start));
			//printf("%f, %f, %f, ", a.x * accel_scale, a.y * accel_scale, a.z * accel_scale);
			//printf("%d, %d, %d\r\n", fData.gyro_x_rate, fData.gyro_y_rate, fData.gyro_z_rate);
			
			if(m_max.x < m.x) m_max.x = m.x;
			if(m_max.y < m.y) m_max.y = m.y;
			if(m_max.z < m.z) m_max.z = m.z;
			
			if(m_min.x > m.x) m_min.x = m.x;
			if(m_min.y > m.y) m_min.y = m.y;
			if(m_min.z > m.z) m_min.z = m.z;
			
			printf("%f/%f   %f/%f   %f/%f\r\n", m_min.x, m_max.x, m_min.y, m_max.y, m_min.z, m_max.z); 
			
			
			//printf("%f, %f, %f \r\n", m.x, m.y, m.z);
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

