#include "include/collision/broadphase/aabb.h"


#include "include/collision/algorithm/gjk.h"
#include "include/dynamics/body.h"
#include "include/geometry/algorithm/2d.h"

namespace Physics2D
{

	bool AABB::isEmpty() const
	{
		return realEqual(width, 0) && realEqual(height, 0) && position.fuzzyEqual({ 0, 0 });
	}
	bool AABB::raycast(const Vector2& start, const Vector2& direction) const
	{
		return raycast(*this, start, direction);
	}

	Vector2 AABB::topLeft() const
	{
		return Vector2{ -width / 2 + position.x , height / 2 + position.y };
	}

	Vector2 AABB::topRight() const
	{
		return Vector2{ width / 2 + position.x , height / 2 + position.y };
	}

	Vector2 AABB::bottomLeft() const
	{
		return Vector2{ -width / 2 + position.x , -height / 2 + position.y };
	}

	Vector2 AABB::bottomRight() const
	{
		return Vector2{ width / 2 + position.x , -height / 2 + position.y };
	}

	bool AABB::collide(const AABB& other) const
	{
		return collide(*this, other);
	}

	void AABB::expand(const real& factor)
	{
		expand(*this, factor);
	}
	

	inline void AABB::clear()
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

	bool AABB::operator==(const AABB& other) const
	{
		return position.fuzzyEqual(other.position) &&
			realEqual(width, other.width) && realEqual(height, other.height);
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
			for (const Vector2& v : polygon->vertices())
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

			break;
		}
		case Shape::Type::Point:
		{
			aabb.width = 1;
			aabb.height = 1;
			break;
		}
		case Shape::Type::Capsule:
		{
			Vector2 p1 = GJK::findFarthestPoint(shape, { 1, 0 });
			Vector2 p2 = GJK::findFarthestPoint(shape, { 0, 1 });
			p1 -= shape.transform;
			p2 -= shape.transform;
			aabb.width = p1.x * 2.0;
			aabb.height = p2.y * 2.0;
			break;
		}
		case Shape::Type::Sector:
		{
			Vector2 p1 = GJK::findFarthestPoint(shape, { 1, 0 });
			Vector2 p2 = GJK::findFarthestPoint(shape, { 0, 1 });
			Vector2 p3 = GJK::findFarthestPoint(shape, { -1, 0 });
			p1 -= shape.transform;
			p2 -= shape.transform;
			p3 -= shape.transform;
			aabb.width = p3.x - p1.x;
			aabb.height = abs(p2.y);
			aabb.position.set({ aabb.width / 2, aabb.height / 2 });
			break;
		}
		}
		aabb.position += shape.transform;
		aabb.expand(factor);
		return aabb;
	}

	AABB AABB::fromBody(Body* body, const real& factor)
	{
		assert(body != nullptr);
		assert(body->shape() != nullptr);
		
		ShapePrimitive primitive;
		primitive.shape = body->shape();
		primitive.rotation = body->rotation();
		primitive.transform = body->position();
		return fromShape(primitive, factor);
	}

	bool AABB::collide(const AABB& src, const AABB& target)
	{
		const Vector2 srcTopLeft = src.topLeft();
		const Vector2 srcBottomRight = src.bottomRight();

		const Vector2 targetTopLeft = target.topLeft();
		const Vector2 targetBottomRight = target.bottomRight();

		return !(srcBottomRight.x < targetTopLeft.x || targetBottomRight.x < srcTopLeft.x || srcTopLeft.y < targetBottomRight.y || targetTopLeft.y < srcBottomRight.y);
	}

	AABB AABB::unite(const AABB& src, const AABB& target, const real& factor)
	{
		if (src.isEmpty())
			return target;

		if (target.isEmpty())
			return src;


		const Vector2 srcTopLeft = src.topLeft();
		const Vector2 srcBottomRight = src.bottomRight();

		const Vector2 targetTopLeft = target.topLeft();
		const Vector2 targetBottomRight = target.bottomRight();

		const real low_x = Math::min(srcTopLeft.x, targetTopLeft.x);
		const real high_x = Math::max(srcTopLeft.x, targetBottomRight.x);
		
		const real low_y = Math::min(srcBottomRight.y, targetBottomRight.y);
		const real high_y = Math::max(srcTopLeft.y, targetTopLeft.y);

		AABB aabb;
		aabb.position.set((low_x + high_x) * 0.5, (low_y + high_y) * 0.5);
		aabb.width = high_x - low_x;
		aabb.height = high_y - low_y;

		aabb.expand(factor);
		return aabb;
	}
	//b is a subset of a
	bool AABB::isSubset(const AABB& a, const AABB& b)
	{

		const Vector2 aTopLeft = a.topLeft();
		const Vector2 aBottomRight = a.bottomRight();

		const Vector2 bTopLeft = b.topLeft();
		const Vector2 bBottomRight = b.bottomRight();

		return aBottomRight.x >= bBottomRight.x && bTopLeft.x >= aTopLeft.x && 
			aTopLeft.y >= bTopLeft.y && bBottomRight.y >= aBottomRight.y;
	}
	void AABB::expand(AABB& aabb, const real& factor)
	{
		aabb.width += factor;
		aabb.height += factor;
	}
	bool AABB::raycast(const AABB& aabb, const Vector2& start, const Vector2& direction)
	{
		auto result = GeometryAlgorithm2D::raycastAABB(start, direction, aabb.topLeft(), aabb.bottomRight());
		if (!result.has_value())
			return false;
		auto [p1, p2] = result.value();
		return GeometryAlgorithm2D::isPointOnAABB(p1, aabb.topLeft(), aabb.bottomRight())
		&& GeometryAlgorithm2D::isPointOnAABB(p2, aabb.topLeft(), aabb.bottomRight());
	}
	void Pair::clear()
	{
		body = nullptr;
		aabb.clear();
	}
}
