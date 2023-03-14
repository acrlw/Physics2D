#ifndef MATH_LINEAR_MATRIX3X3_H
#define MATH_LINEAR_MATRIX3X3_H
#include "common.h"
#include "math.h"
#include "vector2.h"
#include "vector3.h"
namespace Physics2D
{

	struct Matrix3x3
	{
        Matrix3x3() = default;
        Matrix3x3(const Matrix3x3& mat);
        Matrix3x3(const Vector3& col1, const Vector3& col2, const Vector3& col3);
        Matrix3x3(const real& col1_x, const real& col1_y, const real& col1_z,
            const real& col2_x, const real& col2_y, const real& col2_z,
            const real& col3_x, const real& col3_y, const real& col3_z);
		Matrix3x3(Matrix3x3&& other) = default;

        Matrix3x3& operator=(const Matrix3x3& rhs);
        Matrix3x3& operator+=(const Matrix3x3& rhs);
        Matrix3x3& operator-=(const Matrix3x3& rhs);
        Matrix3x3& operator*=(const real& factor);
        Matrix3x3& operator/=(const real& factor);

        Vector3 row1()const;
        Vector3 row2()const;
        Vector3 row3()const;

        real& e11();
        real& e12();
        real& e13();

        real& e21();
        real& e22();
        real& e23();

        real& e31();
        real& e32();
        real& e33();

        Matrix3x3& set(const real& col1_x, const real& col1_y, const real& col1_z,
            const real& col2_x, const real& col2_y, const real& col2_z,
            const real& col3_x, const real& col3_y, const real& col3_z);
        Matrix3x3& set(const Vector3& col1, const Vector3& col2, const Vector3& col3);
        Matrix3x3& set(const Matrix3x3& other);
        Matrix3x3& clear();

        Vector3 multiply(const Vector3& rhs)const;
        Matrix3x3& multiply(const Matrix3x3& rhs);
        real determinant()const;
        Matrix3x3& transpose();
        Matrix3x3& invert();

        static Matrix3x3 skewSymmetricMatrix(const Vector3& v);
        static Matrix3x3 identityMatrix();
        static Matrix3x3 multiply(const Matrix3x3& lhs, const Matrix3x3& rhs);
        static Vector3 multiply(const Matrix3x3& lhs, const Vector3& rhs);
        static real determinant(const Matrix3x3& mat);
        static bool invert(Matrix3x3& mat);

		Vector3 column1;
		Vector3 column2;
		Vector3 column3;
	};
}
#endif
