
#ifndef PX4_FLOW_H
#define PX4_FLOW_H

//PX4 Flow�̃f�[�^��������
#include <string.h>

#include "i2c.h"
#include "rc_io.h"
#include "fram.h"
#include "param.h"

#define PX4F_ADD 0x42
#define PX4F_BAUD 400000

void px4f_init(flow_data *f);

void px4f_update(void);

void px4f_get_raw(uint8_t raw[]);

void calc_flow(void);

Vector3f px4f_get_gyro(void);

#endif
