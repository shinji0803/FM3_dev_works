
#ifndef MYMATH_H
#define MYMATH_H

/* Math functions for AHRS */

#include <Math.h>

#include "param.h"

float	Vector_Dot_Product(const Vector3f *v1, const Vector3f *v2);
void	Vector_Cross_Product(Vector3f *out, const Vector3f *v1, const Vector3f *v2);
void	Vector_Scale(Vector3f *out, const Vector3f *v, float scale);
void	Vector_Add(Vector3f *out, const Vector3f *v1, const Vector3f *v2);
void	Matrix_Multiply(const Matrix3f *m1, const Matrix3f *m2, Matrix3f *out);
void	Matrix_Vector_Multiply(const Matrix3f *m, const Vector3f *v, Vector3f *out);
void	init_rotation_matrix(Matrix3f *m, float yaw, float pitch, float roll);


#endif