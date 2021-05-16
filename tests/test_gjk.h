#pragma once
#include "include/physics2d.h"
#include "include/collision/algorithm/mpr.h"
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
			spa.transform.set(6, -8);
			spa.rotation = 45;

			spb.shape = &b;
			spb.rotation = -45;
			spb.transform.set(6, 8);
			fmt::print("p: {}\n", b.center());

			auto [isCollide, simplex] = GJK::gjk(spa, spb);
			simplex = GJK::epa(spa, spb, simplex);
			auto info = GJK::dumpInfo(simplex);

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
		void testMpr()
		{
			Rectangle rectangle;
			rectangle.set(1, 1);
			ShapePrimitive shape1, shape2;
			shape1.shape = &rectangle;
			shape2.shape = &rectangle;
			shape1.rotation = 45;
			shape1.transform.set(1, 1);
			shape2.rotation = 37;
			shape2.transform.set(1.5, 2);
			auto [centerToOrigin, simplex] = MPR::discover(shape1, shape2);
			auto [isColliding, finalSimplex] = MPR::refine(shape1, shape2, simplex, centerToOrigin);
			fmt::print("collide:{}\n", isColliding);
			fmt::print("A:{}, B:{}, result:{}\n", finalSimplex.vertices[1].pointA, finalSimplex.vertices[1].pointB, finalSimplex.vertices[1].result);
			fmt::print("A:{}, B:{}, result:{}\n", finalSimplex.vertices[2].pointA, finalSimplex.vertices[2].pointB, finalSimplex.vertices[2].result);
		}

		void testSAT()
		{
			Rectangle rectangle;
			rectangle.set(1, 1);
			ShapePrimitive shape1, shape2;
			shape1.shape = &rectangle;
			shape2.shape = &rectangle;
			shape1.rotation = 45;
			shape1.transform.set(1, 1);
			shape2.rotation = 37;
			shape2.transform.set(3, 1);
			SATResult result = SAT::polygonVsPolygon(shape2, shape1);
			fmt::print("collide:{}, normal:{}, penetration:{}\n", result.isColliding, result.normal, result.penetration);
			fmt::print("point pair: {}, {}\n", result.pointPair.pointA, result.pointPair.pointB);
		}
	};
}
