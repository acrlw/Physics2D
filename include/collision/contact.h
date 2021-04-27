#ifndef PHYSICS2D_CONTACT_H
#define PHYSICS2D_CONTACT_H

#include "include/geometry/shape.h"
#include "include/collision/algorithm/gjk.h"
namespace Physics2D
{
	struct ContactEdge
	{
		Vector2 point1;
		Vector2 point2;
	};
	class ContactGenerator
	{
	public:
		static std::optional<std::vector<PointPair>> clip(const ContactEdge& lhs, const ContactEdge& rhs, const bool& exchange = false)
		{
			std::vector<PointPair> result;
			auto project = [](const Vector2& a, const Vector2& b, const Vector2& c)
			{
				return !((c - a).dot(b - a) < 0 || (c - b).dot(a - b) < 0);
			};

			auto push = [&result, &exchange](PointPair& pair)
			{
				if (result.size() < 2)
				{
					if (exchange)
					{
						Vector2 temp = pair.pointA;
						pair.pointA = pair.pointB;
						pair.pointB = temp;
					}
					if (std::find(std::begin(result), std::end(result), pair) == std::end(result))
					{
						result.emplace_back(pair);
						return true;
					}
				}
				return false;
			};
			
			if(project(lhs.point1, lhs.point2, rhs.point1))
			{
				PointPair pair;
				pair.pointA = GeometryAlgorithm2D::pointToLineSegment(lhs.point1, lhs.point2, rhs.point1);
				pair.pointB = rhs.point1;
				push(pair);
			}
			
			if (project(lhs.point1, lhs.point2, rhs.point2))
			{
				PointPair pair;
				pair.pointA = GeometryAlgorithm2D::pointToLineSegment(lhs.point1, lhs.point2, rhs.point2);
				pair.pointB = rhs.point2;
				push(pair);
			}
			
			if (project(rhs.point1, rhs.point2, lhs.point1))
			{
				PointPair pair;
				pair.pointA = lhs.point1;
				pair.pointB = GeometryAlgorithm2D::pointToLineSegment(rhs.point1, rhs.point2, lhs.point1);
				push(pair);
			}
			
			if (project(rhs.point1, rhs.point2, lhs.point2))
			{
				PointPair pair;
				pair.pointA = lhs.point2;
				pair.pointB = GeometryAlgorithm2D::pointToLineSegment(rhs.point1, rhs.point2, lhs.point2);
				push(pair);
			}
			
			return result.empty() ? std::nullopt : std::optional<std::vector<PointPair>>(result);
		}
		static std::optional<std::vector<PointPair>> generate(const ShapePrimitive& shape, const ContactEdge& edge, const Vector2& source, const PenetrationInfo& info, const bool& exchange = false)
		{
			
			if (shape.shape->type() != Shape::Type::Polygon)
				return std::nullopt;
			
			
			auto adjacent = [](const Polygon& polygon, const Vector2& source)
			{
				auto target = std::find_if(std::begin(polygon.vertices()), std::end(polygon.vertices()), [=](const Vector2& element)
					{
						return element.fuzzyEqual(source);
					});

				decltype(target) previous, next;

				
				if (target == std::end(polygon.vertices()))
					next = polygon.vertices().begin() + 1;
				else
					next = std::next(target, 1);
				
				
				if (target == std::begin(polygon.vertices()))
					previous = polygon.vertices().end() - 2;
				else
					previous = std::prev(target, 1);
				
				return std::make_tuple(*previous, *next);
			};
			
			const Vector2 src = Matrix2x2(-shape.rotation).multiply(source - shape.transform);
			Vector2 target;
			auto [previous, next] = adjacent(*dynamic_cast<Polygon*>(shape.shape), src);
			
			previous = Matrix2x2(shape.rotation).multiply(previous) + shape.transform;
			next = Matrix2x2(shape.rotation).multiply(next) + shape.transform;

			if (abs((source - previous).dot(info.normal)) < Constant::GeometryEpsilon)
			{
				target = previous;
			}
			else if (abs((source - next).dot(info.normal)) < Constant::GeometryEpsilon)
			{
				target = next;
			}
			else
				return std::nullopt;
			
			return clip({ source, target }, edge, exchange);
		}
	};
}
#endif
