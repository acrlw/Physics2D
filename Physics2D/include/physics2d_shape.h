#ifndef PHYSICS2D_SHAPE_H
#define PHYSICS2D_SHAPE_H
#include "physics2d_linear.h"
#include "physics2d_common.h"
#include "physics2d_algorithm_2d.h"

namespace Physics2D
{
	class PHYSICS2D_API Shape
	{
	public:
		enum class Type
		{
			Polygon,
			Edge,
			Capsule,
			Circle,
			Ellipse
		};

		Type type() const
		{
			return m_type;
		}

		virtual void scale(const real& factor) = 0;

		virtual ~Shape()
		{
		};
		virtual bool contains(const Vector2& point, const real& epsilon = Constant::GeometryEpsilon) = 0;
		virtual Vector2 center() const = 0;

	protected:
		Type m_type;
	};

	struct PHYSICS2D_API Transform
	{
		//refer https://docs.unity3d.com/ScriptReference/Transform.html
		Vector2 position;
		real rotation = 0;
		real scale = 1.0f;

		Vector2 translatePoint(const Vector2& source) const
		{
			return Matrix2x2(rotation).multiply(source) * scale + position;
		}

		Vector2 inverseTranslatePoint(const Vector2& source) const
		{
			return Matrix2x2(-rotation).multiply(source - position) / scale;
		}

		Vector2 inverseRotatePoint(const Vector2& point) const
		{
			return Matrix2x2(-rotation).multiply(point);
		}
	};

	/**
	 * \brief Basic Shape Description Primitive. Including shape and transform.
	 */
	struct PHYSICS2D_API ShapePrimitive
	{
		ShapePrimitive() = default;
		Shape* shape = nullptr;
		Transform transform;

		bool contains(const Vector2& point, const real& epsilon = Constant::GeometryEpsilon) const
		{
			if (shape == nullptr)
				return false;
			return shape->contains(transform.inverseTranslatePoint(point), epsilon);
		}
	};
}
#endif
