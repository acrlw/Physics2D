#include "include/collision/broadphase/aabb.h"

namespace Physics2D
{
	bool AABB::collide(const AABB& other) const
	{
		return collide(*this, other);
	}

	void AABB::scale(const number& factor)
	{
		AABB::scale(*this, factor);
	}

	AABB AABB::fromShape(const ShapePrimitive& shape, const number& factor)
	{
		AABB aabb;
		aabb.position = shape.transform;
		switch (shape.shape->type())
		{
			case Shape::Type::Polygon:
			{
				const Polygon* polygon = dynamic_cast<Polygon*>(shape.shape);
				aabb.position += polygon->center();
				number max_x = FLT_MIN, max_y = FLT_MIN, min_x = FLT_MAX, min_y = FLT_MAX;
				for(const Vector2& vertex: polygon->vertices())
				{
					if (max_x < vertex.x)
						max_x = vertex.x;
					
					if (min_x > vertex.x)
						min_x = vertex.x;

					if (max_y < vertex.y)
						max_y = vertex.y;

					if (min_y > vertex.y)
						min_y = vertex.y;
				}
				aabb.width = abs(max_x - min_x);
				aabb.height = abs(max_y - min_y);
				break;
			}
			case Shape::Type::Ellipse:
			{
				const Ellipse* ellipse = dynamic_cast<Ellipse*>(shape.shape);
				break;
			}
			case Shape::Type::Circle:
			{
				const Circle* circle = dynamic_cast<Circle*>(shape.shape);
				break;
			}
			case Shape::Type::Edge:
			{
				const Edge* edge = dynamic_cast<Edge*>(shape.shape);
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
				break;
			}
		}
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
	void AABB::scale(AABB& aabb, const number& factor)
	{
		aabb.width *= factor;
		aabb.height *= factor;
	}
}
