#ifndef PHYSICS2D_LINEAR_MATRIX3X3_H
#define PHYSICS2D_LINEAR_MATRIX3X3_H
#include "include/common/common.h"
#include "include/math/math.h"
#include "include/math/linear/vector2.h"
#include "include/math/linear/vector3.h"
namespace Physics2D
{

	struct Matrix3x3
	{
		Matrix3x3() = default;
		Matrix3x3(const Matrix3x3& mat)
		{
			column1 = mat.column1;
			column2 = mat.column2;
			column3 = mat.column3;
		}
		Matrix3x3(const Vector3& col1, const Vector3& col2, const Vector3& col3)
		{
			column1 = col1;
			column2 = col2;
			column3 = col3;
		}
		Matrix3x3(const real& col1_x, const real& col1_y, const real& col1_z,
			const real& col2_x, const real& col2_y, const real& col2_z,
			const real& col3_x, const real& col3_y, const real& col3_z)
		{
			column1.set(col1_x, col1_y, col1_z);
			column2.set(col2_x, col2_y, col2_z);
			column3.set(col3_x, col3_y, col3_z);
		}
		Matrix3x3(Matrix3x3&& other) = default;
		Matrix3x3& operator=(const Matrix3x3& rhs)
		{
			column1 = rhs.column1;
			column2 = rhs.column2;
			column3 = rhs.column3;
			return *this;
		}

		Matrix3x3& operator+=(const Matrix3x3& rhs)
		{
			column1 += rhs.column1;
			column2 += rhs.column2;
			column3 += rhs.column3;
			return *this;
		}

		Matrix3x3& operator-=(const Matrix3x3& rhs)
		{
			column1 -= rhs.column1;
			column2 -= rhs.column2;
			column3 -= rhs.column3;
			return *this;
		}

		Matrix3x3& operator*=(const real& factor)
		{
			column1 *= factor;
			column2 *= factor;
			column3 *= factor;
			return *this;
		}

		Matrix3x3& operator/=(const real& factor)
		{
			assert(!realEqual(factor, 0));
			column1 /= factor;
			column2 /= factor;
			column3 /= factor;
			return *this;
		}
		Vector3 row1()const
		{
			return Vector3(column1.x, column2.x, column3.x);
		}
		Vector3 row2()const
		{
			return Vector3(column1.y, column2.y, column3.y);
		}
		Vector3 row3()const
		{
			return Vector3(column1.z, column2.z, column3.z);
		}
		real determinant()const
		{
			return Matrix3x3::determinant(*this);
		}

		Matrix3x3& transpose()
		{
			realSwap(column1.y, column2.x);
			realSwap(column1.z, column3.x);
			realSwap(column2.z, column3.y);
			return *this;
		}

		Matrix3x3& invert()
		{
			Matrix3x3::invert(*this);
			return *this;
		}

		Matrix3x3& clear()
		{
			column1.clear();
			column2.clear();
			column3.clear();
			return *this;
		}

		Matrix3x3& set(const real& col1_x, const real& col1_y, const real& col1_z,
			const real& col2_x, const real& col2_y, const real& col2_z,
			const real& col3_x, const real& col3_y, const real& col3_z)
		{
			column1.set(col1_x, col1_y, col1_z);
			column2.set(col2_x, col2_y, col2_z);
			column3.set(col3_x, col3_y, col3_z);
			return *this;
		}

		Matrix3x3& set(const Vector3& col1, const Vector3& col2, const Vector3& col3)
		{
			column1 = col1;
			column2 = col2;
			column3 = col3;
			return *this;
		}

		Matrix3x3& set(const Matrix3x3& other)
		{
			column1 = other.column1;
			column2 = other.column2;
			column3 = other.column3;
			return *this;
		}

		Vector3 multiply(const Vector3& rhs)const
		{
			return Matrix3x3::multiply(*this, rhs);
		}

		Matrix3x3& multiply(const Matrix3x3& rhs)
		{
			*this = Matrix3x3::multiply(*this, rhs);
			return *this;
		}
		static Matrix3x3 skewSymmetricMatrix(const Vector3& v)
		{
			return Matrix3x3(
				0, v.z, -v.y,
				-v.z, 0, v.x,
				v.y, -v.x, 0);
		}
		static Matrix3x3 identityMatrix()
		{
			return Matrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1);
		}
		static Matrix3x3 multiply(const Matrix3x3& lhs, const Matrix3x3& rhs)
		{
			return Matrix3x3(Matrix3x3::multiply(lhs, rhs.column1),
				Matrix3x3::multiply(lhs, rhs.column2),
				Matrix3x3::multiply(lhs, rhs.column3));
		}
		static Vector3 multiply(const Matrix3x3& lhs, const Vector3& rhs)
		{
			return Vector3(lhs.column1.x * rhs.x + lhs.column2.x * rhs.y + lhs.column3.x * rhs.z,
				lhs.column1.y * rhs.x + lhs.column2.y * rhs.y + lhs.column3.y * rhs.z,
				lhs.column1.z * rhs.x + lhs.column2.z * rhs.y + lhs.column3.z * rhs.z);
		}
		static real determinant(const Matrix3x3& mat)
		{
			return mat.column1.x * Vector2::crossProduct(mat.column2.y, mat.column2.z, mat.column3.y, mat.column3.z) +
				mat.column2.x * Vector2::crossProduct(mat.column3.y, mat.column3.z, mat.column1.y, mat.column1.z) +
				mat.column3.x * Vector2::crossProduct(mat.column1.y, mat.column1.z, mat.column2.y, mat.column2.z);
		}
		static bool invert(Matrix3x3& mat)
		{
			const real det = mat.determinant();
			if (realEqual(det, 0.0f))
				return false;

			const real det11 = Vector2::crossProduct(mat.column2.y, mat.column2.z, mat.column3.y, mat.column3.z);
			const real det12 = Vector2::crossProduct(mat.column2.x, mat.column2.z, mat.column3.x, mat.column3.z) * -1;
			const real det13 = Vector2::crossProduct(mat.column2.x, mat.column2.y, mat.column3.x, mat.column3.y);

			const real det21 = Vector2::crossProduct(mat.column1.y, mat.column1.z, mat.column3.y, mat.column3.z) * -1;
			const real det22 = Vector2::crossProduct(mat.column1.x, mat.column1.z, mat.column3.x, mat.column3.z);
			const real det23 = Vector2::crossProduct(mat.column1.x, mat.column1.y, mat.column3.x, mat.column3.y) * -1;

			const real det31 = Vector2::crossProduct(mat.column1.y, mat.column1.z, mat.column2.y, mat.column2.z);
			const real det32 = Vector2::crossProduct(mat.column1.x, mat.column1.z, mat.column2.x, mat.column2.z) * -1;
			const real det33 = Vector2::crossProduct(mat.column1.x, mat.column1.y, mat.column2.x, mat.column2.y);

			mat.set(det11, det12, det13, det21, det22, det23, det31, det32, det33);
			mat.transpose();
			mat /= det;
			return true;
		}
		Vector3 column1;
		Vector3 column2;
		Vector3 column3;
	};
}
#endif
