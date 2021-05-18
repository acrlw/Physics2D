#include "include/collision/broadphase/aabb.h"


#include "include/dynamics/body.h"
#include "include/geometry/algorithm/2d.h"

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

	void AABB::clear()
	{
		position.clear();
		width = 0.0;
		height = 0.0;
	}

	AABB& AABB::unite(const AABB& other)
	{
		*this = unite(*this, other);
		return *this;
	}

	real AABB::surfaceArea() const
	{
		return (width + height) * 2;
	}

	real AABB::volume() const
	{
		return width * height;
	}

	bool AABB::isSubset(const AABB& other) const
	{
		return isSubset(other, *this);
	}

	AABB AABB::fromShape(const ShapePrimitive& shape, const real& factor)
	{
		AABB aabb;
		switch (shape.shape->type())
		{
			case Shape::Type::Polygon:
			{
				const Polygon* polygon = dynamic_cast<Polygon*>(shape.shape.get());
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
				aabb.position.set((max_x + min_x) * 0.5, (max_y + min_y) * 0.5);
				break;
			}
			case Shape::Type::Ellipse:
			{
				const Ellipse* ellipse = dynamic_cast<Ellipse*>(shape.shape.get());

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
				const Circle* circle = dynamic_cast<Circle*>(shape.shape.get());
				aabb.width = circle->radius() * 2;
				aabb.height = circle->radius() * 2;
				break;
			}
			case Shape::Type::Edge:
			{
				const Edge* edge = dynamic_cast<Edge*>(shape.shape.get());
				aabb.width = abs(edge->startPoint().x - edge->endPoint().x);
				aabb.height = abs(edge->startPoint().y - edge->endPoint().y);
				aabb.position.set(edge->startPoint().x + edge->endPoint().x, edge->startPoint().y + edge->endPoint().y);
				aabb.position *= 0.5;
				break;
			}
			case Shape::Type::Curve:
			{
				const Curve* curve = dynamic_cast<Curve*>(shape.shape.get());
				
				break;
			}
			case Shape::Type::Point:
			{
				const Point* curve = dynamic_cast<Point*>(shape.shape.get());
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

	AABB AABB::fromBody(Body* body, const real& factor)
	{
		assert(body != nullptr);
		assert(body->shape() != nullptr);
		
		ShapePrimitive primitive;
		primitive.shape = body->shape();
		primitive.rotation = body->angle();
		primitive.transform = body->position();
		return fromShape(primitive, factor);
	}

	bool AABB::collide(const AABB& src, const AABB& target)
	{
		const real src_low_x = (-src.width * 0.5) + src.position.x;
		const real src_high_x = (src.width * 0.5) + src.position.x;


		const real src_low_y = (-src.height * 0.5) + src.position.y;
		const real src_high_y = (src.height * 0.5) + src.position.y;


		const real target_low_x = (-target.width * 0.5) + target.position.x;
		const real target_high_x = (target.width * 0.5) + target.position.x;


		const real target_low_y = (-target.height * 0.5) + target.position.y;
		const real target_high_y = (target.height * 0.5) + target.position.y;

		return !(src_high_x < target_low_x || target_high_x < src_low_x || src_high_y < target_low_y || target_high_y < src_low_y);
	}

	AABB AABB::unite(const AABB& src, const AABB& target, const real& factor)
	{
		if (src.isEmpty())
			return target;

		if (target.isEmpty())
			return src;
		
		const real src_low_x = (-src.width * 0.5) + src.position.x;
		const real src_high_x = (src.width * 0.5) + src.position.x;


		const real src_low_y = (-src.height * 0.5) + src.position.y;
		const real src_high_y = (src.height * 0.5) + src.position.y;


		const real target_low_x = (-target.width * 0.5) + target.position.x;
		const real target_high_x = (target.width * 0.5) + target.position.x;


		const real target_low_y = (-target.height * 0.5) + target.position.y;
		const real target_high_y = (target.height * 0.5) + target.position.y;

		const real low_x = Math::min(src_low_x, target_low_x);
		const real high_x = Math::max(src_high_x, target_high_x);
		
		const real low_y = Math::min(src_low_y, target_low_y);
		const real high_y = Math::max(src_high_y, target_high_y);

		AABB aabb;
		aabb.position.set((low_x + high_x) * 0.5, (low_y + high_y) * 0.5);
		aabb.width = high_x - low_x;
		aabb.height = high_y - low_y;

		aabb.scale(factor);
		return aabb;
	}
	void AABB::scale(AABB& aabb, const real& factor)
	{
		aabb.width *= factor;
		aabb.height *= factor;
	}
	//b is a subset of a
	bool AABB::isSubset(const AABB& a, const AABB& b)
	{
		const real a_low_x = (-a.width * 0.5) + a.position.x;
		const real a_high_x = (a.width * 0.5) + a.position.x;


		const real b_low_x = (-b.width * 0.5) + b.position.x;
		const real b_high_x = (b.width * 0.5) + b.position.x;


		const real a_low_y = (-a.height * 0.5) + a.position.y;
		const real a_high_y = (a.height * 0.5) + a.position.y;


		const real b_low_y = (-b.height * 0.5) + b.position.y;
		const real b_high_y = (b.height * 0.5) + b.position.y;

		return a_high_x >= b_high_x && b_low_x >= a_low_x && a_high_y >= b_high_y && b_low_y >= a_low_y;
	}
}
