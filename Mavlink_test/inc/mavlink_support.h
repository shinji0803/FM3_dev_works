
#ifndef _MAVLINK_SUP_DEF_H_
#define _MAVLINK_SUP_DEF_H_

#include "hw_config.h"
#include "UartDev.h"
#include "mavlink.h"

int32_t Mavlink_port_init(uint8_t ch, uint32_t baudrate);

int32_t mavlink_tx(void *data, int32_t *size);

void mavlink_rx_check(void);

void mavlink_rx_check_test(void);

#endif