#ifndef PHYSICS2D_RENDERER_H
#define PHYSICS2D_RENDERER_H

#include <QPainter>

#include "include/geometry/shape.h"
#include "include/dynamics/world.h"

namespace Physics2D
{
	//class Renderer
	//{
	//public:
	//	virtual void renderPolygon(const ShapePrimitive& shape) = 0;
	//	virtual void renderRectangle(const ShapePrimitive& shape) = 0;
	//	virtual void renderEllipse(const ShapePrimitive& shape) = 0;
	//	virtual void renderEdge(const ShapePrimitive& shape) = 0;
	//	virtual void renderCenterPoint(const ShapePrimitive& shape) = 0;
	//	virtual void renderCircle(const ShapePrimitive& shape) = 0;
	//	virtual void renderPoint(const Vector2& pos) = 0;
	//	virtual void renderLine(const Vector2& p1, const Vector2& p2) = 0;
	//};
	class Renderer
	{
	public:
		Renderer(){}
		static void render(QPainter* painter, World* world, const QPen& pen);
	private:
	};
}
#endif