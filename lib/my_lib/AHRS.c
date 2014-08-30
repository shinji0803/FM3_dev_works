
#include "AHRS.h"

static Matrix3f dcm_matrix = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

static Vector3f gyroOffset;

static void init_rotation_matrix(Matrix3f *m, float yaw, float pitch, float roll);
static uint8_t check_gyro_calib_data(Vector3f *g);

static Vector3f rawG, rawA, rawM;
static Vector3f scaledG, scaledA, scaledM;
static float mag_heading = 0.0f;

//DCM Parameter
static Vector3f omega = { 0.0f, 0.0f, 0.0f};
static Vector3f omega_vector = { 0.0f, 0.0f, 0.0f};
static Vector3f omega_P = { 0.0f, 0.0f, 0.0f};
static Vector3f omega_I = { 0.0f, 0.0f, 0.0f};

static Vector3f errorRollPitch = { 0.0f, 0.0f, 0.0f};
static Vector3f errorYaw = { 0.0f, 0.0f, 0.0f};

void AHRS_Init(void)
{
	time.calibrate = 1;
	
	Vector3f temp = { 0.0f, 0.0f, 0.0f};
	Vector3f temp1, temp2;
	Vector3f sum = { 0.0f, 0.0f, 0.0f};
	
	const Vector3f xAxis = {1.0f, 0.0f, 0.0f};
	
	int32_t sum_count = 0;

	/* Acc and Mag enable */
	LSM303DLH_Init();
	/* Gyro enable */
	WMP_init();
	
	wait(1000);
	
	/* Gyro scale calibration */
	cputs("Gyro Calibration");
	
	while(1){
		sum_count = 0;
		while(1){
#ifdef USE_PX4F_GYRO		
			px4f_get_gyro(&temp); // Gyro get from PX4FLOW
#else
			WMP_get_raw_gyro(&temp); // Gyro get from WiiMotion+
#endif			
			if(check_gyro_calib_data(&temp) != 0) break;
			sum.x += temp.x;
			sum.y += temp.y;
			sum.z += temp.z;
			sum_count ++;
			
			cputs(".");
			wait(200);
			if(sum_count >= 10) break;
		}
		
		if(sum_count >= 10) break;
		else{
			cputs("\r\nDo not move IMU.\r\nRecalibrate");
			sum.x = sum.y = sum.z = 0.0f;
			sum_count = 0;
			wait(500);
		}
		
	}
	printf("\r\n");
	gyroOffset.x = sum.x / 10.0f; 
	gyroOffset.y = sum.y / 10.0f; 
	gyroOffset.z = sum.z / 10.0f;
	
	printf("Gyro Offset ( %f, %f, %f)\r\n", gyroOffset.x, gyroOffset.y, gyroOffset.z);
	
	//Initialize DCM Matrix
	Vector3f att;
	//Get Roll and Pitch
	AHRS_read_imu();
	AHRS_get_acc(&temp);
	att.y = atan2(temp.x, sqrt(temp.y * temp.y + temp.z * temp.z));
	Vector_Cross_Product(&temp1, &temp, &xAxis);
	Vector_Cross_Product(&temp2, &xAxis, &temp1);
	att.x = atan2( temp2.y, temp2.z);
	//Get Yaw
	att.z = ToRad(AHRS_heading((Vector3f){0,1,0}));
	init_rotation_matrix(&dcm_matrix, att.z, att.y, att.x);
	
	time.calibrate = 0;
}


static uint8_t check_gyro_calib_data(Vector3f *g)
{
	uint8_t error_count = 0;
	
	if(g->x > GYRO_CALIB_HIGH || g->x < GYRO_CALIB_LOW) error_count ++;
	if(g->y > GYRO_CALIB_HIGH || g->y < GYRO_CALIB_LOW) error_count ++;
	if(g->z > GYRO_CALIB_HIGH || g->z < GYRO_CALIB_LOW) error_count ++;
	return error_count;
}

void AHRS_read_imu(void)
{
	uint8_t slow_mode;
	
	/* Gyro read */
#ifdef USE_PX4F_GYRO	
	px4f_get_gyro(&rawG);
#else
	WMP_get_raw_gyro(&rawG);
	WMP_get_gyro_mode(&slow_mode);
#endif

	rawG.x = rawG.x - gyroOffset.x;
	rawG.y = -(rawG.y - gyroOffset.y); // Pitch reversed
	rawG.z = rawG.z - gyroOffset.z;
	
	if(((slow_mode >> 2) & 0x01) != 1) rawG.x *= 4.545454f; // When high speed mode, multiply value by (2000/440)
	if(((slow_mode >> 1) & 0x01) != 1) rawG.y *= 4.545454f;
	if(((slow_mode >> 0) & 0x01) != 1) rawG.z *= 4.545454f;
	/* Gyro read end */
	
	/* Acc read */
	readAcc(&rawA);
	/* Acc read end */
	
	/* Mag read */
	readMag(&rawM);
	/* Mag read end */
}

void AHRS_dcm_update(float dt)
{
	Matrix3f dcm_update_matrix;
	Matrix3f temp_matrix = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
	
	AHRS_get_gyro(&scaledG);
	AHRS_get_acc(&scaledA);
	
	Vector_Add(&omega, &scaledG, &omega_I);
	Vector_Add(&omega_vector, &omega, &omega_P);
	
	/*
	dcm_update_matrix.a.x = 0;
	dcm_update_matrix.a.y = -dt * omega_vector.z;
	dcm_update_matrix.a.z = dt * omega_vector.y;
	dcm_update_matrix.b.x = dt * omega_vector.z;
	dcm_update_matrix.b.y = 0;
	dcm_update_matrix.b.z = -dt * omega_vector.x;
	dcm_update_matrix.c.x = -dt * omega_vector.y;
	dcm_update_matrix.c.y = dt * omega_vector.x;
	dcm_update_matrix.c.z = 0;
	*/
	
	dcm_update_matrix.a.x = 0;
	dcm_update_matrix.a.y = -dt * omega_vector.z;
	dcm_update_matrix.a.z = dt * omega_vector.x;
	dcm_update_matrix.b.x = dt * omega_vector.z;
	dcm_update_matrix.b.y = 0;
	dcm_update_matrix.b.z = -dt * omega_vector.y;
	dcm_update_matrix.c.x = -dt * omega_vector.x;
	dcm_update_matrix.c.y = dt * omega_vector.y;
	dcm_update_matrix.c.z = 0;
	
	Matrix_Multiply(&dcm_matrix, &dcm_update_matrix, &temp_matrix);
	
	dcm_matrix.a.x += temp_matrix.a.x;
	dcm_matrix.a.y += temp_matrix.a.y;
	dcm_matrix.a.z += temp_matrix.a.z;
	dcm_matrix.b.x += temp_matrix.b.x;
	dcm_matrix.b.y += temp_matrix.b.y;
	dcm_matrix.b.z += temp_matrix.b.z;
	dcm_matrix.c.x += temp_matrix.c.x;
	dcm_matrix.c.y += temp_matrix.c.y;
	dcm_matrix.c.z += temp_matrix.c.z;
}

void AHRS_dcm_normalize(void)
{
	float error = 0.0f;
	Matrix3f temp;
	float renorm = 0.0f;
	
	error = -Vector_Dot_Product(&dcm_matrix.a, &dcm_matrix.b) * 0.5f;
	
	Vector_Scale(&temp.a, &dcm_matrix.b, error);
	Vector_Scale(&temp.b, &dcm_matrix.a, error);
	
	Vector_Add(&temp.a, &temp.a, &dcm_matrix.a);
	Vector_Add(&temp.b, &temp.b, &dcm_matrix.b);
	
	Vector_Cross_Product(&temp.c, &temp.a, &temp.b);
	
	renorm= 0.5f *(3 - Vector_Dot_Product(&temp.a, &temp.a));
	Vector_Scale(&dcm_matrix.a, &temp.a, renorm);
	
	renorm= 0.5f *(3 - Vector_Dot_Product(&temp.b, &temp.b));
	Vector_Scale(&dcm_matrix.b, &temp.b, renorm);
	
	renorm= 0.5f *(3 - Vector_Dot_Product(&temp.c, &temp.c));
	Vector_Scale(&dcm_matrix.c, &temp.c, renorm);
}

void AHRS_drift_correction(void)
{
	float mag_heading_x;
	float mag_heading_y;
	float errorCourse;

	static Vector3f scaled_omega_P = {0.0f, 0.0f, 0.0f};
	static Vector3f scaled_omega_I = {0.0f, 0.0f, 0.0f};
	float accel_magnitude;
	float accel_weight;
	
	/* calculate heading from mag */
	mag_heading = ToRad(AHRS_heading((Vector3f){0,1,0}));
	
	/* Roll and Pitch */
	accel_magnitude = sqrt(scaledA.x * scaledA.x + scaledA.y * scaledA.y + scaledA.z * scaledA.z);
	accel_magnitude = accel_magnitude / GRAVITY;
	accel_weight = constrain(1 - 2 * abs(1 - accel_magnitude), 0, 1);
	
	Vector_Cross_Product(&errorRollPitch, &scaledA,&dcm_matrix.c);
	Vector_Scale(&omega_P, &errorRollPitch, Kp_ROLLPITCH * accel_weight);
	
	Vector_Scale(&scaled_omega_I, &errorRollPitch, Ki_ROLLPITCH * accel_weight);
	Vector_Add(&omega_I, &omega_I, &scaled_omega_I);
	
	/* YAW */
	mag_heading_x = cos(mag_heading);
	mag_heading_y = sin(mag_heading);
	errorCourse = (dcm_matrix.a.x * mag_heading_y) - (dcm_matrix.b.x * mag_heading_x);
	Vector_Scale(&errorYaw, &dcm_matrix.c, errorCourse);
  
	Vector_Scale(&scaled_omega_P, &errorYaw, Kp_YAW);
	Vector_Add(&omega_P, &omega_P, &scaled_omega_P);
  
	Vector_Scale(&scaled_omega_I, &errorYaw, Ki_YAW);
	Vector_Add(&omega_I, &omega_I, &scaled_omega_I);
}

void AHRS_get_gyro(Vector3f *g)
{
	//AHRS_get_raw_gyro(g);
	
	g->x = ToRad(rawG.x / GYRO_SCALE);
	g->y = ToRad(rawG.y / GYRO_SCALE);
	g->z = ToRad(rawG.z / GYRO_SCALE);
}

void AHRS_get_raw_gyro(Vector3f *g)
{
	g->x = rawG.x;
	g->y = rawG.y;
	g->z = rawG.z;
}

void AHRS_get_acc(Vector3f *a)
{
	//AHRS_get_raw_acc(a);
	a->x = (rawA.x - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
	a->y = (rawA.y - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
	a->z = (rawA.z - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;
}

void AHRS_get_raw_acc(Vector3f *a)
{
	a->x = rawA.x;
	a->y = rawA.y;
	a->z = rawA.z;
}

void AHRS_get_mag(Vector3f *m)
{
	m->x = rawM.x;
	m->y = rawM.y;
	m->z = rawM.z;
}

void AHRS_get_raw_mag(Vector3f *m)
{
	m->x = rawM.x;
	m->y = rawM.y;
	m->z = rawM.z;
}


void init_rotation_matrix(Matrix3f *m, float yaw, float pitch, float roll)
{	
	float c1 = cos(roll);
	float s1 = sin(roll);
	float c2 = cos(pitch);
	float s2 = sin(pitch);
	float c3 = cos(yaw);
	float s3 = sin(yaw);

	// Euler angles, right-handed, intrinsic, XYZ convention
	// (which means: rotate around body axes Z, Y', X'') 
	m->a.x = c2 * c3;
	m->a.y = c3 * s1 * s2 - c1 * s3;
	m->a.z = s1 * s3 + c1 * c3 * s2;
	
	m->b.x = c2 * s3;
	m->b.y = c1 * c3 + s1 * s2 * s3;
	m->b.z = c1 * s2 * s3 - c3 * s1;
	
	m->c.x = -s2;
	m->c.y = c2 * s1;
	m->c.z = c1 * c2;
}

void AHRS_get_euler(Vector3f *att)
{
	att->x = -asin(dcm_matrix.c.x);
	att->y = atan2(dcm_matrix.c.y, dcm_matrix.c.z);
	att->z = atan2(dcm_matrix.b.x, dcm_matrix.a.x);
}

/* Move from LSM303DLH library */
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
float AHRS_heading(Vector3f from)
{
	Vector3f temp_a, temp_m;
	AHRS_get_raw_acc(&temp_a);
	AHRS_get_raw_mag(&temp_m);
    // shift and scale
    temp_m.x = (temp_m.x - m_min.x) / (m_max.x - m_min.x) * 2 - 1.0;
    temp_m.y = (temp_m.y - m_min.y) / (m_max.y - m_min.y) * 2 - 1.0;
    temp_m.z = (temp_m.z - m_min.z) / (m_max.z - m_min.z) * 2 - 1.0;

    //Vector3f temp_a = a;
    // normalize
    Vector_Normalize(&temp_a);
    //vector_normalize(&m);

    // compute E and N
    Vector3f E;
    Vector3f N;
    Vector_Cross_Product(&E, &temp_m, &temp_a);
    Vector_Normalize(&E);
    Vector_Cross_Product(&N, &temp_a, &E);
	
    // compute heading
	float heading = (atan2(Vector_Dot_Product(&E, &from), Vector_Dot_Product(&N, &from)) * 180 / M_PI);
    if (heading < 0) heading += 360.0;
	return heading;
}
	
	
