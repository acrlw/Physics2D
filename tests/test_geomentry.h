#pragma once
#include "tests/test.h"
#include "include/physics2d.h"
namespace Physics2D
{
	class TestGeometryAlgorithm : public Test
	{
	public:
		TestGeometryAlgorithm()
		{
			m_name = "geometry algorithm test";
		}
		void run() override
		{
			testIntersection();
		}
		void testIntersection()
		{
			fmt::print("-----line intersection-----\n");
			Vector2 p1(0, 5);
			Vector2 p2(5, 10);
			Vector2 p3(0, 0);
			Vector2 p4(5, 5);
			auto result = GraphicsAlgorithm2D::lineSegmentIntersection(p1, p2, p3, p4);
			if(result.has_value())
			{
				fmt::print("line intersection: {}\n", GraphicsAlgorithm2D::lineSegmentIntersection(p1, p2, p3, p4).value());
			}
			else
			{
				fmt::print("no intersection.\n");
			}
			auto p = GraphicsAlgorithm2D::raycast({ 1, 0 }, { 3, 1 }, {11, 0}, {0, 6});
			if(p.has_value())
			{
				fmt::print("ray cast on point: {}\n", p.value());
			}
			else
			{
				fmt::print("ray cast false\n");
			}
			
			
		}
	};
}
