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
			auto result = GeometryAlgorithm2D::lineSegmentIntersection(p1, p2, p3, p4);
			if(result.has_value())
			{
				fmt::print("line intersection: {}\n", GeometryAlgorithm2D::lineSegmentIntersection(p1, p2, p3, p4).value());
			}
			else
			{
				fmt::print("no intersection.\n");
			}
			auto p = GeometryAlgorithm2D::raycast({ 1, 0 }, { 3, 1 }, {11, 0}, {0, 6});
			if(p.has_value())
			{
				fmt::print("ray cast on point: {}\n", p.value());
			}
			else
			{
				fmt::print("ray cast false\n");
			}
			Vector2 rot_p = GeometryAlgorithm2D::rotate({ 2, 5 }, { 5, 4 }, -45);
			fmt::print("(2, 5) rotate around (5, 4) ,rotate 45 degree {}\n", rot_p);
			auto value = GeometryAlgorithm2D::calculateInscribedCircle({ -4, 4 }, { -2, -2 }, { -4, -4 });
			if(value.has_value())
			{
				auto [point, radius] = value.value();
				fmt::print("point at {}, radius: {}\n", point, radius);
			}
			else
			{
				fmt::print("no inscribed circle.\n");
			}
			auto circum = GeometryAlgorithm2D::calculateCircumcircle({ -4, 4 }, { -2, -2 }, { -4, -4 });
			if(circum.has_value())
			{
				auto [point, radius] = circum.value();
				fmt::print("point at {}, radius: {}\n", point, radius);
			}
			else
			{
				fmt::print("Failed to calculate circum-circle.\n");
			}
			bool is_polygon = GeometryAlgorithm2D::isConvexPolygon({{-2, 4},{-4, 0},{-2, -4},{6, -4},{0, 2},{-2, 4}});
			fmt::print("is polygon convex:{}\n",is_polygon);
			std::vector<Vector2> vertices = GeometryAlgorithm2D::grahamScan({
				{2, 2}, {6, 2},{4, 0},{2, 4},
				{-2, 2},{8, -2},{0, 6},{-4, -5},
				{-9, -1}, {-1, -10} , {0, -4}, {-10, 6}});
			fmt::print("convex hull vertices: \n");
			for (const Vector2& vertex : vertices)
				fmt::print("{} \n", vertex);
			fmt::print("\n");
		}
	};
}
