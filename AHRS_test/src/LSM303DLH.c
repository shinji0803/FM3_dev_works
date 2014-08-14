
#include "LSM303DLH.h"
#include "i2c.h"
#include "param.h"

#include <math.h>

// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address, 
// and sets the last bit correctly based on reads and writes
#define ACC_ADDRESS (0x30 >> 1)
#define MAG_ADDRESS (0x3C >> 1)

static Vector3f a; // accelerometer readings
static Vector3f m; // magnetometer readings
static Vector3f m_max; // maximum magnetometer values, used for calibration
static Vector3f m_min; // minimum magnetometer values, used for calibration

// Public Methods //////////////////////////////////////////////////////////////

// Turns on the LSM303DLH's accelerometer and magnetometers and places them in normal
// mode.
void LSM303DLH_Init(void)
{	
	// Enable Accelerometer
	// 0x27 = 0b00100111
	// Normal power mode, all axes enabled
	//writeAccReg(LSM303DLH_CTRL_REG1_A, 0x27);
	writeAccReg(LSM303DLH_CTRL_REG1_A, 0x37);
	
	// Enable Magnetometer
	// 0x00 = 0b00000000
	// Continuous conversion mode
	writeMagReg(LSM303DLH_MR_REG_M, 0x00);
}

// Writes an accelerometer register
void writeAccReg(uint8_t reg, uint8_t value)
{
	int32_t size;
	uint8_t data[2] = { reg, value};
	
	i2c->Cfg.SlaveAddr = ACC_ADDRESS;
	size = 2;
	i2c->DataTx(&data, &size);
}

// Reads an accelerometer register
uint8_t readAccReg(uint8_t reg)
{
	int32_t size;
	uint8_t data = reg;
	
	i2c->Cfg.SlaveAddr = ACC_ADDRESS;
	size = 1;
	i2c->DataTx(&data, &size);
	
	size = 1;
	i2c->DataRx(&data, &size);
	
	return data;
}

// Writes a magnetometer register
void writeMagReg(uint8_t reg, uint8_t value)
{
	int32_t size;
	uint8_t data[2] = { reg, value};
	
	i2c->Cfg.SlaveAddr = MAG_ADDRESS;
	size = 2;
	i2c->DataTx(&data, &size);
}

// Reads a magnetometer register
uint8_t readMagReg(uint8_t reg)
{
	int32_t size;
	uint8_t data = reg;
	
	i2c->Cfg.SlaveAddr = MAG_ADDRESS;
	size = 1;
	i2c->DataTx(&data, &size);
	
	size = 1;
	i2c->DataRx(&data, &size);
	
	return data;
}

// Reads the 3 accelerometer channels and stores them in vector a
Vector3f readAcc(void)
{
	int32_t size;
	uint8_t data[8];
	
	i2c->Cfg.SlaveAddr = ACC_ADDRESS;
	// assert the MSB of the address to get the accelerometer 
	// to do slave-transmit subaddress updating.
	size = 1;
	data[0] = LSM303DLH_OUT_X_L_A | (1 << 7);
	i2c->DataTx( &data, &size); 
	
	size = 6;
	i2c->DataRx( &data, &size); 
	
	uint8_t xha = data[1];
	uint8_t xla = data[0];
	uint8_t yha = data[3];
	uint8_t yla = data[2];
	uint8_t zha = data[5];
	uint8_t zla = data[4];
	
	a.x = (int16_t)((xha << 8) | xla);
	a.y = (int16_t)((yha << 8) | yla);
	a.z = (int16_t)((zha << 8) | zla);
	
	return a;
}

// Reads the 3 magnetometer channels and stores them in vector m
Vector3f readMag(void)
{
	int32_t size;
	uint8_t data[8];
	
	i2c->Cfg.SlaveAddr = MAG_ADDRESS;
	size = 1;
	data[0] = LSM303DLH_OUT_X_H_M;
	i2c->DataTx( &data, &size); 
	
	size = 6;
	i2c->DataRx( &data, &size); 


	uint8_t xhm = data[0];
	uint8_t xlm = data[1];
	uint8_t yhm = data[2];
	uint8_t ylm = data[3];
	uint8_t zhm = data[4];
	uint8_t zlm = data[5];

	m.x = (int16_t)(xhm << 8 | xlm);
	m.y = (int16_t)(yhm << 8 | ylm);
	m.z = (int16_t)(zhm << 8 | zlm);
	
	return m;
}

// Reads all 6 channels of the LSM303DLH and stores them in the object variables
void read(void)
{
	readAcc();
	readMag();
}

// Returns the number of degrees from the -Y axis that it
// is pointing.
/*
int heading(void)
{
	return heading((vector){0,-1,0});
}
*/

// Returns the number of degrees from the From vector projected into
// the horizontal plane is away from north.
// 
// Description of heading algorithm: 
// Shift and scale the magnetic reading based on calibration data to
// to find the North vector. Use the acceleration readings to
// determine the Down vector. The cross product of North and Down
// vectors is East. The vectors East and North form a basis for the
// horizontal plane. The From vector is projected into the horizontal
// plane and the angle between the projected vector and north is
// returned.
int heading(Vector3f from)
{
    // shift and scale
    m.x = (m.x - m_min.x) / (m_max.x - m_min.x) * 2 - 1.0;
    m.y = (m.y - m_min.y) / (m_max.y - m_min.y) * 2 - 1.0;
    m.z = (m.z - m_min.z) / (m_max.z - m_min.z) * 2 - 1.0;

    Vector3f temp_a = a;
    // normalize
    vector_normalize(&temp_a);
    //vector_normalize(&m);

    // compute E and N
    Vector3f E;
    Vector3f N;
    vector_cross(&m, &temp_a, &E);
    vector_normalize(&E);
    vector_cross(&temp_a, &E, &N);
	
    // compute heading
    //int heading = round(atan2(vector_dot(&E, &from), vector_dot(&N, &from)) * 180 / M_PI);
	int heading = (int)(atan2(vector_dot(&E, &from), vector_dot(&N, &from)) * 180 / M_PI);
    if (heading < 0) heading += 360;
	return heading;
}

void vector_cross( const Vector3f *a, const Vector3f *b, Vector3f *out)
{
  out->x = a->y * b->z - a->z * b->y;
  out->y = a->z * b->x - a->x * b->z;
  out->z = a->x * b->y - a->y * b->x;
}

float vector_dot( const Vector3f *a, const Vector3f *b)
{
  return a->x * b->x + a->y * b->y + a->z * b->z;
}

void vector_normalize(Vector3f *a)
{
  float mag = sqrt(vector_dot(a,a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}
