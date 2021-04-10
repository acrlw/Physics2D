#include "include/collision/broadphase/aabb.h"

namespace Physics2D
{
	bool AABB::collide(const AABB& other) const
	{
		return collide(*this, other);
	}

	void AABB::scale(const real& factor)
	{
		scale(*this, factor);
	}

	AABB AABB::fromShape(const ShapePrimitive& shape, const real& factor)
	{
		AABB aabb;
		switch (shape.shape->type())
		{
			case Shape::Type::Polygon:
			{
				const Polygon* polygon = dynamic_cast<Polygon*>(shape.shape);
				real max_x = -FLT_MAX, max_y = -FLT_MAX, min_x = FLT_MAX, min_y = FLT_MAX;
				for(const Vector2& v: polygon->vertices())
				{
					const Vector2 vertex = Matrix2x2(shape.rotation).multiply(v);
					if (max_x < vertex.x)
						max_x = vertex.x;
					
					if (min_x > vertex.x)
						min_x = vertex.x;

					if (max_y < vertex.y)
						max_y = vertex.y;

					if (min_y > vertex.y)
						min_y = vertex.y;
				}
				aabb.width = max_x - min_x;
				aabb.height = max_y - min_y;
				aabb.position.set((max_x + min_x) / 2, (max_y + min_y) / 2);
				break;
			}
			case Shape::Type::Ellipse:
			{
				const Ellipse* ellipse = dynamic_cast<Ellipse*>(shape.shape);
				Vector2 top, left, bottom, right;
					
				Vector2 top_dir{ 0, 1 };
				Vector2 left_dir{ -1, 0 };
				Vector2 bottom_dir{ 0, -1 };
				Vector2 right_dir{ 1, 0 };
					
				top_dir = Matrix2x2(-shape.rotation).multiply(top_dir);
				left_dir = Matrix2x2(-shape.rotation).multiply(left_dir);
				bottom_dir = Matrix2x2(-shape.rotation).multiply(bottom_dir);
				right_dir = Matrix2x2(-shape.rotation).multiply(right_dir);
					
				top = GeometryAlgorithm2D::calculateEllipseProjectionPoint(ellipse->A(), ellipse->B(), top_dir);
				left = GeometryAlgorithm2D::calculateEllipseProjectionPoint(ellipse->A(), ellipse->B(), left_dir);
				bottom = GeometryAlgorithm2D::calculateEllipseProjectionPoint(ellipse->A(), ellipse->B(), bottom_dir);
				right = GeometryAlgorithm2D::calculateEllipseProjectionPoint(ellipse->A(), ellipse->B(), right_dir);
					
				top = Matrix2x2(shape.rotation).multiply(top);
				left = Matrix2x2(shape.rotation).multiply(left);
				bottom = Matrix2x2(shape.rotation).multiply(bottom);
				right = Matrix2x2(shape.rotation).multiply(right);
					
				aabb.height = top.y - bottom.y;
				aabb.width = right.x - left.x;
				break;
			}
			case Shape::Type::Circle:
			{
				const Circle* circle = dynamic_cast<Circle*>(shape.shape);
				aabb.width = circle->radius() * 2;
				aabb.height = circle->radius() * 2;
				break;
			}
			case Shape::Type::Edge:
			{
				const Edge* edge = dynamic_cast<Edge*>(shape.shape);
				aabb.width = abs(edge->startPoint().x - edge->endPoint().x);
				aabb.height = abs(edge->startPoint().y - edge->endPoint().y);
				aabb.position.set(edge->startPoint().x + edge->endPoint().x, edge->startPoint().y + edge->endPoint().y);
				aabb.position /= 2;
				break;
			}
			case Shape::Type::Curve:
			{
				const Curve* curve = dynamic_cast<Curve*>(shape.shape);
				
				break;
			}
			case Shape::Type::Point:
			{
				const Point* curve = dynamic_cast<Point*>(shape.shape);
				aabb.width = 1;
				aabb.height = 1;
				break;
			}
		}
		aabb.position += shape.transform;
		aabb.width *= factor;
		aabb.height *= factor;
		return aabb;
	}

	bool AABB::collide(const AABB& src, const AABB& target)
	{
		return false;
	}

	AABB AABB::unite(const AABB& src, const AABB& target)
	{
		return AABB();
	}
	void AABB::scale(AABB& aabb, const real& factor)
	{
		aabb.width *= factor;
		aabb.height *= factor;
	}
}
