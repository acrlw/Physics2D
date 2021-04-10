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

	AABB AABB::unite(const AABB& other)const
	{
		return AABB::unite(*this, other);
	}

	AABB AABB::fromShape(const ShapePrimitive& shape, const real& factor)
	{
		AABB aabb;
		switch (shape.shape->type())
		{
			case Shape::Type::Polygon:
			{
				const Polygon* polygon = dynamic_cast<Polygon*>(shape.shape);
				real max_x = Constant::NegativeMin, max_y = Constant::NegativeMin, min_x = Constant::Max, min_y = Constant::Max;
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
				aabb.width = abs(max_x - min_x);
				aabb.height = abs(max_y - min_y);
				aabb.position.set((max_x + min_x) / 2, (max_y + min_y) / 2);
				break;
			}
			case Shape::Type::Ellipse:
			{
				const Ellipse* ellipse = dynamic_cast<Ellipse*>(shape.shape);

				Vector2 top_dir{ 0, 1 };
				Vector2 left_dir{ -1, 0 };
				Vector2 bottom_dir{ 0, -1 };
				Vector2 right_dir{ 1, 0 };
					
				top_dir = Matrix2x2(-shape.rotation).multiply(top_dir);
				left_dir = Matrix2x2(-shape.rotation).multiply(left_dir);
				bottom_dir = Matrix2x2(-shape.rotation).multiply(bottom_dir);
				right_dir = Matrix2x2(-shape.rotation).multiply(right_dir);
					
				Vector2 top = GeometryAlgorithm2D::calculateEllipseProjectionPoint(ellipse->A(), ellipse->B(), top_dir);
				Vector2 left = GeometryAlgorithm2D::calculateEllipseProjectionPoint(ellipse->A(), ellipse->B(), left_dir);
				Vector2 bottom = GeometryAlgorithm2D::calculateEllipseProjectionPoint(ellipse->A(), ellipse->B(), bottom_dir);
				Vector2 right = GeometryAlgorithm2D::calculateEllipseProjectionPoint(ellipse->A(), ellipse->B(), right_dir);
					
				top = Matrix2x2(shape.rotation).multiply(top);
				left = Matrix2x2(shape.rotation).multiply(left);
				bottom = Matrix2x2(shape.rotation).multiply(bottom);
				right = Matrix2x2(shape.rotation).multiply(right);
					
				aabb.height = abs(top.y - bottom.y);
				aabb.width = abs(right.x - left.x);
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
		const real src_low_x = (-src.width / 2) + src.position.x;
		const real src_high_x = (src.width / 2) + src.position.x;


		const real src_low_y = (-src.height / 2) + src.position.y;
		const real src_high_y = (src.height / 2) + src.position.y;


		const real target_low_x = (-target.width / 2) + target.position.x;
		const real target_high_x = (target.width / 2) + target.position.x;


		const real target_low_y = (-target.height / 2) + target.position.y;
		const real target_high_y = (target.height / 2) + target.position.y;

		return !(src_high_x < target_low_x || target_high_x < src_low_x || src_high_y < target_low_y || target_high_y < src_low_y);
	}

	AABB AABB::unite(const AABB& src, const AABB& target)
	{
		const real src_low_x = (-src.width / 2) + src.position.x;
		const real src_high_x = (src.width / 2) + src.position.x;


		const real src_low_y = (-src.height / 2) + src.position.y;
		const real src_high_y = (src.height / 2) + src.position.y;


		const real target_low_x = (-target.width / 2) + target.position.x;
		const real target_high_x = (target.width / 2) + target.position.x;


		const real target_low_y = (-target.height / 2) + target.position.y;
		const real target_high_y = (target.height / 2) + target.position.y;

		const real low_x = min(src_low_x, target_low_x);
		const real high_x = max(src_high_x, target_high_x);
		
		const real low_y = min(src_low_y, target_low_y);
		const real high_y = max(src_high_y, target_high_y);

		AABB aabb;
		aabb.position.set((low_x + high_x) / 2, (low_y + high_y) / 2);
		aabb.width = high_x - low_x;
		aabb.height = high_y - low_y;

		return aabb;
	}
	void AABB::scale(AABB& aabb, const real& factor)
	{
		aabb.width *= factor;
		aabb.height *= factor;
	}
}
