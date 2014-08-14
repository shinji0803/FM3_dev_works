
/* This file is based on AHRS Firmware */

#include "myMath.h"

// Computes the dot product of two vectors
float Vector_Dot_Product(const Vector3f *v1, const Vector3f *v2)
{
	float result = 0;
	result += v1->x * v2->x;
	result += v1->y * v2->y;
	result += v1->z * v2->z;
    return result; 
}

// Computes the cross product of two vectors
// out has to different from v1 and v2 (no in-place)!
void Vector_Cross_Product(Vector3f *out, const Vector3f *v1, const Vector3f *v2)
{
	out->x = (v1->y * v2->z) - (v1->z * v2->y);
	out->y = (v1->z * v2->x) - (v1->x * v2->z);
	out->z = (v1->x * v2->y) - (v1->y * v2->x);
}

// Multiply the vector by a scalar
void Vector_Scale(Vector3f *out, const Vector3f *v, float scale)
{
	out->x = v->x * scale;
	out->y = v->y * scale; 
	out->z = v->z * scale; 	
}

// Adds two vectors
void Vector_Add(Vector3f *out, const Vector3f *v1, const Vector3f *v2)
{
	out->x = v1->x + v2->x;
	out->y = v1->y + v2->y;
	out->z = v1->z + v2->z;
}

// Multiply two 3x3 matrices: out = m1 * m2
// out has to different from a and b (no in-place)!
void Matrix_Multiply(const Matrix3f *m1, const Matrix3f *m2, Matrix3f *out)
{
	out->a.x = m1->a.x * m2->a.x + m1->a.y * m2->b.x + m1->a.z * m2->c.x;
	out->a.y = m1->a.x * m2->a.y + m1->a.y * m2->b.y + m1->a.z * m2->c.y;
	out->a.z = m1->a.x * m2->a.z + m1->a.y * m2->b.z + m1->a.z * m2->c.z;
	out->b.x = m1->b.x * m2->a.x + m1->b.y * m2->b.x + m1->b.z * m2->c.x;
	out->b.y = m1->b.x * m2->a.y + m1->b.y * m2->b.y + m1->b.z * m2->c.y;
	out->b.z = m1->b.x * m2->a.z + m1->b.y * m2->b.z + m1->b.z * m2->c.z;
	out->c.x = m1->c.x * m2->a.x + m1->c.y * m2->b.x + m1->c.z * m2->c.x;
	out->c.y = m1->c.x * m2->a.y + m1->c.y * m2->b.y + m1->c.z * m2->c.y;
	out->c.z = m1->c.x * m2->a.z + m1->c.y * m2->b.z + m1->c.z * m2->c.z;
}

// Multiply 3x3 matrix with vector: out = a * b
// out has to different from b (no in-place)!
void Matrix_Vector_Multiply(const Matrix3f *m, const Vector3f *v, Vector3f *out)
{
	out->x = m->a.x * v->x + m->a.y * v->y + m->a.z * v->z;
	out->y = m->b.x * v->x + m->b.y * v->y + m->b.z * v->z;
	out->z = m->c.x * v->x + m->c.y * v->y + m->c.z * v->z;
}

// Init rotation matrix using euler angles
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
