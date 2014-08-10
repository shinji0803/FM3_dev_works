
#include <math.h>

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

#define TENTIMES(x) do { x; x; x; x; x; x; x; x; x; x; } while (0)
#define FIFTYTIMES(x) do { TENTIMES(x); TENTIMES(x); TENTIMES(x); TENTIMES(x); TENTIMES(x); } while (0)

#define TIMEIT(name, op, count) do { \
	uint32_t us_end, us_start; \
	us_start = get_micros(); \
	for (uint8_t i=0; i<count; i++) { \
		FIFTYTIMES(op);				\
	} \
	us_end = get_micros(); \
	printf("%-10s %7.2f usec/call\r\n", name, (float)(us_end-us_start)/(count*50.0f)); \
} while (0)

volatile float v_f = 1.0;
volatile float v_out;
volatile double v_d = 1.0;
volatile double v_out_d;
volatile uint32_t v_32 = 1;
volatile uint32_t v_out_32 = 1;
volatile uint16_t v_16 = 1;
volatile uint16_t v_out_16 = 1;
volatile uint8_t v_8 = 1;
volatile uint8_t v_out_8 = 1;
volatile uint8_t mbuf1[128], mbuf2[128];
volatile uint64_t v_64 = 1;
volatile uint64_t v_out_64 = 1;

int32_t main(void){
	
	//‰Šú‰»ŠJŽn
	conio_init(57600UL);
	
	Init_timer();
	InitLED();
	rcin_enable(0);
	
	//Init_i2c();
	//Init_fram();
	Init_DT();
	
	printf("Initialize OK.\r\n");
	//‰Šú‰»I—¹
	
	wait(500);
	
	v_f = 1+(get_micros() % 5);
	v_out = 1+(get_micros() % 3);

	v_32 = 1+(get_micros() % 5);
	v_out_32 = 1+(get_micros() % 3);

	v_16 = 1+(get_micros() % 5);
	v_out_16 = 1+(get_micros() % 3);

	v_8 = 1+(get_micros() % 5);
	v_out_8 = 1+(get_micros() % 3);


	printf("Operation timings:");
	printf("Note: timings for some operations are very data dependent\r\n");

	TIMEIT("nop", asm volatile("nop"::), 255);

	TIMEIT("micros()", get_micros(), 200);
	TIMEIT("millis()", get_millis(), 200);

	TIMEIT("fadd", v_out += v_f, 100);
	TIMEIT("fsub", v_out -= v_f, 100);
	TIMEIT("fmul", v_out *= v_f, 100);
	TIMEIT("fdiv /=", v_out /= v_f, 100);
    TIMEIT("fdiv 2/x", v_out = 2.0f/v_f, 100);

	TIMEIT("dadd", v_out_d += v_d, 100);
	TIMEIT("dsub", v_out_d -= v_d, 100);
	TIMEIT("dmul", v_out_d *= v_d, 100);
	TIMEIT("ddiv", v_out_d /= v_d, 100);

	TIMEIT("sin()", v_out = sinf(v_f), 20);
	TIMEIT("cos()", v_out = cosf(v_f), 20);
	TIMEIT("tan()", v_out = tanf(v_f), 20);
	TIMEIT("acos()", v_out = acosf(v_f * 0.2), 20);
	TIMEIT("asin()", v_out = asinf(v_f * 0.2), 20);
	TIMEIT("atan2()", v_out = atan2f(v_f * 0.2, v_f * 0.3), 20);
	TIMEIT("sqrt()",v_out = sqrtf(v_f), 20);

	TIMEIT("iadd8", v_out_8 += v_8, 100);
	TIMEIT("isub8", v_out_8 -= v_8, 100);
	TIMEIT("imul8", v_out_8 *= v_8, 100);
	TIMEIT("idiv8", v_out_8 /= v_8, 100);

	TIMEIT("iadd16", v_out_16 += v_16, 100);
	TIMEIT("isub16", v_out_16 -= v_16, 100);
	TIMEIT("imul16", v_out_16 *= v_16, 100);
	TIMEIT("idiv16", v_out_16 /= v_16, 100);

	TIMEIT("iadd32", v_out_32 += v_32, 100);
	TIMEIT("isub32", v_out_32 -= v_32, 100);
	TIMEIT("imul32", v_out_32 *= v_32, 100);
	TIMEIT("idiv32", v_out_32 /= v_32, 100);

	TIMEIT("iadd64", v_out_64 += v_64, 100);
	TIMEIT("isub64", v_out_64 -= v_64, 100);
	TIMEIT("imul64", v_out_64 *= v_64, 100);
	TIMEIT("idiv64", v_out_64 /= v_64, 100);

	TIMEIT("memcpy128", memcpy((void*)mbuf1, (const void *)mbuf2, sizeof(mbuf1)), 20);
	TIMEIT("memset128", memset((void*)mbuf1, 1, sizeof(mbuf1)), 20);
	TIMEIT("wait(1)", wait(1), 5);
	
	while(1); 
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

