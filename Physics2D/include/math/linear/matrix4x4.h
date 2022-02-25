#ifndef MATH_LINEAR_MATRIX4X4_H
#define MATH_LINEAR_MATRIX4X4_H
#include "../../common/common.h"
#include "../../math/math.h"
#include "../../math/linear/matrix3x3.h"
#include "../../math/linear/vector4.h"
namespace Physics2D
{
	struct Matrix4x4
	{
		Matrix4x4() = default;
		Matrix4x4(const Vector4& col1, const Vector4& col2, const Vector4& col3, const Vector4& col4);

		Matrix4x4(const real& col1_x, const real& col1_y, const real& col1_z, const real& col1_w,
			const real& col2_x, const real& col2_y, const real& col2_z, const real& col2_w,
			const real& col3_x, const real& col3_y, const real& col3_z, const real& col3_w,
			const real& col4_x, const real& col4_y, const real& col4_z, const real& col4_w);

		Matrix4x4(const Matrix3x3& mat);
		Matrix4x4(const Matrix4x4& mat) = default;

		Matrix4x4(Matrix4x4&& other) = default;

        Matrix4x4& operator=(const Matrix3x3& rhs);
        Matrix4x4& operator=(const Matrix4x4& rhs);

        Matrix4x4& operator+=(const Matrix4x4& rhs);
        Matrix4x4& operator-=(const Matrix4x4& rhs);
        Matrix4x4& operator*=(const real& factor);
        Matrix4x4& operator/=(const real& factor);

        Vector4 row1()const;
        Vector4 row2()const;
        Vector4 row3()const;
        Vector4 row4()const;

        real& e11();
        real& e12();
        real& e13();
        real& e14();

        real& e21();
        real& e22();
        real& e23();
        real& e24();

        real& e31();
        real& e32();
        real& e33();
        real& e34();

        real& e41();
        real& e42();
        real& e43();
        real& e44();

        Matrix4x4& set(const real& col1_x, const real& col1_y, const real& col1_z, const real& col1_w,
            const real& col2_x, const real& col2_y, const real& col2_z, const real& col2_w,
            const real& col3_x, const real& col3_y, const real& col3_z, const real& col3_w,
            const real& col4_x, const real& col4_y, const real& col4_z, const real& col4_w);

        Matrix4x4& set(const Vector4& col1, const Vector4& col2, const Vector4& col3, const Vector4& col4);
        Matrix4x4& set(const Matrix4x4& other);
        Matrix4x4& set(const Matrix3x3& other);

        Matrix4x4& clear();

        Vector4 multiply(const Vector4& rhs)const;
        Matrix4x4& multiply(const Matrix4x4& rhs);
        real determinant()const;
        Matrix4x4& transpose();
        Matrix4x4& invert();
        
        static Matrix4x4 identityMatrix();
        static Matrix4x4 multiply(const Matrix4x4& lhs, const Matrix4x4& rhs);
        static Vector4 multiply(const Matrix4x4& lhs, const Vector4& rhs);
        static real determinant(const Matrix4x4& mat);
        static bool invert(Matrix4x4& mat);

		Vector4 column1;
		Vector4 column2;
		Vector4 column3;
		Vector4 column4;
	};
}
#endif