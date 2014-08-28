
#include "mavlink_support.h"

static UartDev_IOB *mavlink;

static uint8_t mav_msg[MAVLINK_MAX_PACKET_LEN];

int32_t Mavlink_port_init(uint8_t ch, uint32_t baudrate)
{
	int32_t result;
	
	// Port setting
	FM3_GPIO->PFR7 = FM3_GPIO->PFR7 | 0x0C;
	FM3_GPIO->EPFR07 = FM3_GPIO->EPFR07 | 0x40000;
	
	// Driver setting
	mavlink = &UartDev[ch];
	
	mavlink->Cfg.BufSize = 256;
	mavlink->Cfg.BaudRate = baudrate;
	mavlink->Cfg.BitOrder = UartDev_BITORDER_LSB;
	
	result = mavlink->Init();
	return result;
}

int32_t Mavlink_tx(void *data, int32_t *size)
{
	return mavlink->BufTx(data, size, UartDev_FLAG_BLOCKING);
}

void Mavlink_rx_check(void)
{
	int32_t size;
	uint8_t receive_buf[8];
	
	if(mavlink->RxAvailable() > 0){
		size = 1;
		mavlink->BufRx(receive_buf, &size, UartDev_FLAG_NONBLOCKING);
		
		if(size == 1){
			if(receive_buf[0] == MAVLINK_STX) printf("\r\n");
			printf("%2x ", receive_buf[0]);
		}
	}
}

void Mavlink_rx_check_test(void)
{
	int32_t size;
	uint8_t receive_buf[8];
	static uint16_t index = 0;
	
	if(mavlink->RxAvailable() > 0){
		size = 1;
		mavlink->BufRx(receive_buf, &size, UartDev_FLAG_NONBLOCKING);
		
		if(size == 1){
			if(receive_buf[0] == MAVLINK_STX){
				if(mav_msg[1] == (index - 6 - 2)){
					for(int count = 0; count < index; count ++) printf("%2x ", mav_msg[count]);
					printf("\r\n");
					index = 0;
				}
			}
			mav_msg[index] = receive_buf[0];
			
			index ++;
		}
	}
}

void Mavlink_printf(const char* format, ...)
{
	va_list args;
	uint8_t convert[256];
	int32_t size = 0;
		
	va_start( args, format);
	vsprintf((char *)convert, format, args);
	va_end(args);
	
	size = strlen((char *)convert);
	mavlink->BufTx(convert, &size, UartDev_FLAG_BLOCKING);
}
