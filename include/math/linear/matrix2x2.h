#ifndef PHYSICS2D_LINEAR_MATRIX2X2
#define PHYSICS2D_LINEAR_MATRIX2X2
#include "include/math/linear/vector2.h"
#include "include/math/math.h"
namespace Physics2D
{

	struct Matrix2x2
	{
		Matrix2x2() = default;
		Matrix2x2(const real& angle)
		{
			setAngle(angle);
		}
		Matrix2x2(const Matrix2x2& mat)
		{
			column1 = mat.column1;
			column2 = mat.column2;
		}
		Matrix2x2(const Vector2& col1, const Vector2& col2)
		{
			column1 = col1;
			column2 = col2;
		}
		Matrix2x2(const real& col1_x, const real& col1_y, const real& col2_x, const real& col2_y)
		{
			column1.set(col1_x, col1_y);
			column2.set(col2_x, col2_y);
		}
		Matrix2x2(Matrix2x2&& other) = default;
		Matrix2x2& operator=(const Matrix2x2& rhs)
		{
			column1 = rhs.column1;
			column2 = rhs.column2;
			return *this;
		}

		Matrix2x2& operator+=(const Matrix2x2& rhs)
		{
			column1 += rhs.column1;
			column2 += rhs.column2;
			return *this;
		}

		Matrix2x2& operator-=(const Matrix2x2& rhs)
		{
			column1 -= rhs.column1;
			column2 -= rhs.column2;
			return *this;
		}

		Matrix2x2& operator*=(const real& factor)
		{
			column1 *= factor;
			column2 *= factor;
			return *this;
		}

		Matrix2x2& operator/=(const real& factor)
		{
			assert(!realEqual(factor, 0));
			column1 /= factor;
			column2 /= factor;
			return *this;
		}

		Matrix2x2 operator+(const Matrix2x2& rhs)const
		{
			return Matrix2x2(column1 + rhs.column1, column2 + rhs.column2);
		}

		Matrix2x2 operator-(const Matrix2x2& rhs)const
		{
			return Matrix2x2(column1 - rhs.column1, column2 - rhs.column2);
		}
		Vector2 row1()const
		{
			return Vector2(column1.x, column2.x);
		}
		Vector2 row2()const
		{
			return Vector2(column1.y, column2.y);
		}
		real determinant()const
		{
			return Matrix2x2::determinant(*this);
		}

		Matrix2x2& transpose()
		{
			realSwap(column1.y, column2.x);
			return *this;
		}

		Matrix2x2& invert()
		{
			Matrix2x2::invert(*this);
			return *this;
		}

		Matrix2x2& multiply(const Matrix2x2& rhs)
		{
			*this = Matrix2x2::multiply(*this, rhs);
			return *this;
		}

		Vector2 multiply(const Vector2& rhs)const
		{
			return Matrix2x2::multiply(*this, rhs);
		}

		Matrix2x2& clear()
		{
			column1.clear();
			column2.clear();
			return *this;
		}

		Matrix2x2& set(const real& col1_x, const real& col1_y, const real& col2_x, const real& col2_y)
		{
			column1.set(col1_x, col1_y);
			column2.set(col2_x, col2_y);
			return *this;
		}

		Matrix2x2& set(const Vector2& col1, const Vector2& col2)
		{
			column1 = col1;
			column2 = col2;
			return *this;
		}

		Matrix2x2& set(const Matrix2x2& other)
		{
			column1 = other.column1;
			column2 = other.column2;
			return *this;
		}

		Matrix2x2& setAngle(const real& angle)
		{
			const real arc = angle * Constant::Pi / 180;
			const real cosarc = cosx(arc);
			const real sinarc = sinx(arc);
			column1.set(cosarc, sinarc);
			column2.set(-sinarc, cosarc);
			return *this;
		}
		static Matrix2x2 skewSymmetricMatrix(const Vector2& r)
		{
			return Matrix2x2(0, -r.y, r.x, 0);
		}
		static Matrix2x2 identityMatrix()
		{
			return Matrix2x2(1, 0, 0, 1);
		}
		static Vector2 multiply(const Matrix2x2& lhs, const Vector2& rhs)
		{
			return Vector2(lhs.column1.x * rhs.x + lhs.column2.x * rhs.y, lhs.column1.y * rhs.x + lhs.column2.y * rhs.y);
		}

		static Matrix2x2 multiply(const Matrix2x2& lhs, const Matrix2x2& rhs)
		{
			return Matrix2x2(lhs.column1.x * rhs.column1.x + lhs.column2.x * rhs.column1.y,
				lhs.column1.y * rhs.column1.x + lhs.column2.y * rhs.column1.y,
				lhs.column1.x * rhs.column2.x + lhs.column2.x * rhs.column2.y,
				lhs.column1.y * rhs.column2.x + lhs.column2.y * rhs.column2.y);
		}
		static real determinant(const Matrix2x2& mat)
		{
			return mat.column1.x * mat.column2.y - mat.column2.x * mat.column1.y;
		}
		static bool invert(Matrix2x2& mat)
		{
			const real det = mat.determinant();

			if (realEqual(det, 0.0f))
				return false;

			realSwap(mat.column1.x, mat.column2.y);
			mat.column1.y *= -1;
			mat.column2.x *= -1;
			mat /= det;
			return true;
		}
		Vector2 column1;
		Vector2 column2;
	};
}
#endif