#pragma once
#include "tests/test.h"
#include "include/math/math.h"
#include "include/common/common.h"

#include "fmt/core.h"
#include "include/math/linear/linear.h"


namespace Physics2D
{
	class TestMath : public Test
	{
	public:
		TestMath()
		{
			m_name = "math testing";
		}
		virtual void run() override
		{
			testVector();
			testMatrix();
			testOthers();
			testGeometryAlgorithm();
		}
		void testVector()
		{
			fmt::print("-----vector2 test-----\n");
			Vector2 v1;
			fmt::print("v1: {}\n", v1);
			
			Vector2 v2(1, 2);
			fmt::print("v2: {}\n", v2);
			
			Vector2 v3 = { 1, 4 };
			fmt::print("v3: {}\n", v3);
			
			Vector2 v4 = v3;
			fmt::print("v4: {}\n", v4);
			
			Vector2 v5;
			fmt::print("v5: {}\n", v5);
			v5.set(1, 2);
			fmt::print("v5.set(1, 2): {}\n", v5);
			v5.set(v3);
			fmt::print("v5.set(v3): {}\n", v5);
			v5.clear();
			fmt::print("v5.clear(): {}\n", v5);
			v5.set(3, 5);
			fmt::print("v5.set(3, 5): {}\n", v5);
			v5.negate();
			fmt::print("v5.negate(): {}\n", v5);
			
			Vector2 v6 = v5.normal();
			fmt::print("v6: {}\n", v6);
			
			v5.normalize();
			fmt::print("v5.normalize(): {}\n", v5);
			
			fmt::print("v5:{} equals v6:{} = {}\n",v5, v6, v5.equal(v6));
			fmt::print("v3:{} dot v5:{} = {}\n", v3, v5, v3.dot(v5));
			fmt::print("v3:{} cross v6:{} = {}\n", v3, v6, v3.cross(v6));
			fmt::print("v3.perpendicular(): {}\n", v3.perpendicular());
			fmt::print("v3:{} swap to v2:{}, v3:{}\n", v3, v2, v3.swap(v3));
			fmt::print("length of v3: {}\n", v3.length());
			fmt::print("length square of v3: {}\n", v3.lengthSquare());
			
			fmt::print("-----vector3 test-----\n");
			Vector3 v7;
			fmt::print("v7: {}\n", v7);
			Vector3 v8(1, 2, 3);
			fmt::print("v8: {}\n", v8);
			Vector3 v9 = { 4, 5, 6 };
			fmt::print("v9: {}\n", v9);
			Vector3 v10 = v9;
			fmt::print("v10: {}\n", v10);
			Vector3 v11(v8);
			fmt::print("v11: {}\n", v11);
			Vector3 v12;
			fmt::print("v12: {}\n", v12);
			v12.set(7, 8, 9);
			fmt::print("v12.set(7, 8, 9): {}\n", v12);
			v7.set(v12);
			fmt::print("v7.set(v12): {}\n", v7);
			fmt::print("v12.cross(v8): {}\n", v12.cross(v8));
			fmt::print("v12.dot(v9): {}\n", v12.dot(v9));
			fmt::print("v12.length(): {}\n", v12.length());
			fmt::print("v12.lengthSquare(): {}\n", v12.lengthSquare());
			fmt::print("v12:{} equal v9:{} = {}\n", v12, v9, v12.equal(v9));
			fmt::print("v12.normal(): {}\n", v12.normal());
			fmt::print("v12.negate(): {}\n", v12.negate());
			fmt::print("v12.swap(v8): {}\n", v12.swap(v8));
		}
		void testMatrix()
		{
			fmt::print("-----matrix2x2 test-----\n");
			Matrix2x2 mat1;
			fmt::print("mat1: {}", mat1);
			Matrix2x2 mat2(1, 2, 3, 4);
			fmt::print("mat2: {}", mat2);
			Matrix2x2 mat3(7, 8, 9, 10);
			fmt::print("mat3: {}", mat3);
			Matrix2x2 mat4(mat2);
			fmt::print("mat4: {}", mat4);
			fmt::print("mat2.multiply(mat3): {}", mat2.multiply(mat3));
			fmt::print("mat2.invert(): {}", mat2.invert());
			fmt::print("mat2.transpose(): {}", mat2.transpose());
			fmt::print("mat2.determinant(): {}\n", mat2.determinant());
			fmt::print("identity matrix:{}", Matrix2x2::identityMatrix());
			fmt::print("-----matrix3x3 test-----\n");

			Matrix3x3 mat5;
			fmt::print("mat5: {}", mat5);
			Matrix3x3 mat6(1, 2, 3, 4, 5, 6, 7, 8, 9);
			fmt::print("mat6: {}", mat6);
			Matrix3x3 mat7 = { 10, 11, 12, 13, 14, 15, 16, 17, 18 };
			fmt::print("mat7: {}", mat7);
			fmt::print("mat6.multiply(mat7): {}", mat6.multiply(mat7));
			Matrix3x3 mat8 = { 1, 2, 9, 3, 8, 4, 7, 5, 6 };
			fmt::print("mat8: {}", mat8);
			fmt::print("mat8.determinant(): {}\n", mat8.determinant());
			fmt::print("mat8.transpose(): {}", mat8.transpose());
			fmt::print("mat8.invert(): {}", mat8.invert());
			fmt::print("mat3x3 identity matrix: {}", Matrix3x3::identityMatrix());


			fmt::print("-----rotation2 matrix test-----\n");
			Matrix2x2 rot1(37);
			Matrix2x2 rot2(-54);
			Vector2 v(5, 0);
			fmt::print("v:{} rotate {} degree, v:{}\n", v, 37, rot1.multiply(v));
			fmt::print("v:{} rotate {} degree, v:{}\n", v, -54, rot2.multiply(v));
		}
		void testOthers()
		{
			fmt::print("-----other test-----\n");
			fmt::print("system method: 1/sqrt(23) = {}\n", 1.0f / sqrt(23));
			fmt::print("fast inverse sqrt method: 1/sqrt(23) = {}\n", Math::fastInverseSqrt<real>(23));
		}
		void testGeometryAlgorithm()
		{
			
		}
	};
}
