
#ifndef _MAVLINK_SUP_DEF_H_
#define _MAVLINK_SUP_DEF_H_

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "hw_config.h"
#include "UartDev.h"
#include "mavlink.h"

int32_t Mavlink_port_init(uint8_t ch, uint32_t baudrate);

int32_t Mavlink_tx(void *data, int32_t *size);

void Mavlink_rx_check(void);

void Mavlink_rx_check_test(void);

void Mavlink_printf(const char* format, ...);
#endif