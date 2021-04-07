#pragma once
#include "include/physics2d.h"
#include "tests/test.h"
namespace Physics2D
{
	class GjkTest : public Test
	{
		GjkTest()
		{
			m_name = "gjk test";
		}
		void run() override
		{
			
		}
		void testPointCollision()
		{
			
		}
		void testPolygonCollision()
		{
			
		}
		void testEllipsePolygon()
		{
			Ellipse a;
			a.set(20, 16);
			Polygon b;
			b.append({ 0,4 });
			b.append({ -4,2 });
			b.append({ -2,-2 });
			b.append({ 2,-4 });
			b.append({ 4,0 });
			b.append({ 0,4 });
			ShapePrimitive spa, spb;
			spa.shape = &a;
			spa.translation.set(6, -8);
			spa.rotation = 45;

			spb.shape = &b;
			spb.rotation = -45;
			spb.translation.set(6, 8);
			fmt::print("p: {}\n", b.center());

			auto [isCollide, simplex] = GJK::gjk(spa, spb);
			simplex = GJK::epa(spa, spb, simplex);
			ContactInfo info = GJK::dumpInfo(spa, spb, simplex);

			auto result = GeometryAlgorithm2D::lineSegmentIntersection({ -4,2 }, { -2,3 }, { -2,0 }, { -3,4 });
			if (result.has_value())
				fmt::print("intersection point:{}\n", result.value());
			else
				fmt::print("no intersection\n");
			auto isOnSegment = GeometryAlgorithm2D::isPointOnSegment({ 0, 4 }, { 4, 0 }, { 1, 3 });
			fmt::print("is on segment:{} \n", isOnSegment);
			auto pl = GeometryAlgorithm2D::shortestLengthPointOfEllipse(10, 8, { 16, -6 });
			fmt::print("p:{}\n", pl);
		}
		void testCircle()
		{
			
		}
		void testEdge()
		{
			
		}
	};
}
