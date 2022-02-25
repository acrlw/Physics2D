#ifndef MATH_LINEAR_MATRIX2X2_H
#define MATH_LINEAR_MATRIX2X2_H
#include "../../math/linear/vector2.h"
#include "../../math/math.h"
namespace Physics2D
{

	struct Matrix2x2
	{
		Matrix2x2() = default;
        Matrix2x2(const real& radian);
        Matrix2x2(const Matrix2x2& mat);
        Matrix2x2(const Vector2& col1, const Vector2& col2);
        Matrix2x2(const real& col1_x, const real& col1_y, const real& col2_x, const real& col2_y);
		Matrix2x2(Matrix2x2&& other) = default;

        Matrix2x2& operator=(const Matrix2x2& rhs);
        Matrix2x2& operator+=(const Matrix2x2& rhs);
        Matrix2x2& operator-=(const Matrix2x2& rhs);
        Matrix2x2& operator*=(const real& factor);
        Matrix2x2& operator/=(const real& factor);
        Matrix2x2 operator+(const Matrix2x2& rhs)const;
        Matrix2x2 operator-(const Matrix2x2& rhs)const;

        Vector2 row1()const;
        Vector2 row2()const;
		
        real& e11();
        real& e12();
		
        real& e21();
        real& e22();

        real determinant()const;
        Matrix2x2& transpose();
        Matrix2x2& invert();
        Matrix2x2& multiply(const Matrix2x2& rhs);
        Vector2 multiply(const Vector2& rhs)const;

        Matrix2x2& clear();
        Matrix2x2& set(const real& col1_x, const real& col1_y, const real& col2_x, const real& col2_y);
        Matrix2x2& set(const Vector2& col1, const Vector2& col2);
        Matrix2x2& set(const Matrix2x2& other);
        Matrix2x2& set(const real& radian);

        static Matrix2x2 skewSymmetricMatrix(const Vector2& r);
        static Matrix2x2 identityMatrix();
        static Vector2 multiply(const Matrix2x2& lhs, const Vector2& rhs);
        static Matrix2x2 multiply(const Matrix2x2& lhs, const Matrix2x2& rhs);
        static real determinant(const Matrix2x2& mat);
        static bool invert(Matrix2x2& mat);

        Vector2 column1;
		Vector2 column2;
	};
}
#endif
