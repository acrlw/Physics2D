#ifndef PHYSICS2D_RENDERER_IMPL_QT_H
#define PHYSICS2D_RENDERER_IMPL_QT_H
#include "include/physics2d.h"
#include "include/render/renderer.h"
#include "QPainter"
#include "QPointF"
#include "QPainterPath"
#include "QPen"
#include "QBrush"
namespace Physics2D
{
	class RendererQtImpl
	{
	public:
		static void renderPoint(QPainter* painter, World* world, const Vector2& point, const QPen& pen)
		{
			assert(painter != nullptr && world != nullptr);
			Vector2 screen_p = world->worldToScreen(point);
			QPen p = pen;
			p.setWidth(6);
			painter->setPen(p);
			painter->drawPoint(QPointF(screen_p.x, screen_p.y));
		}
		static void renderLine(QPainter* painter, World* world, const Vector2& p1, const Vector2& p2, const QPen& pen)
		{
			assert(painter != nullptr && world != nullptr);
			Vector2 screen_p1 = world->worldToScreen(p1);
			Vector2 screen_p2 = world->worldToScreen(p2);
			painter->setPen(pen);
			painter->drawLine(QPointF(screen_p1.x, screen_p1.y), QPointF(screen_p2.x, screen_p2.y));
		}
		static void renderPolygon(QPainter* painter, World* world, const ShapePrimitive& shape, const QPen& pen)
		{
			assert(painter != nullptr && world != nullptr);
			assert(shape.shape->type() == Shape::Type::Polygon);
			Vector2 position = world->worldToScreen(shape.transform);
			renderPoint(painter, world, shape.transform, pen);
			QPen center(Qt::gray, 8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
			renderPoint(painter, world, Matrix2x2(shape.rotation).multiply(dynamic_cast<Polygon*>(shape.shape)->center()) + shape.transform, center);
			QPolygonF polygon;
			QColor color = pen.color();
			color.setAlphaF(0.15f);
			QBrush brush(color);

			for(const Vector2& point: dynamic_cast<Polygon*>(shape.shape)->vertices())
			{
				const Vector2 world_p = Matrix2x2(shape.rotation).multiply(point) + shape.transform;
				const Vector2 screen_p = world->worldToScreen(world_p);
				polygon << QPointF(screen_p.x, screen_p.y);
			}
			QPainterPath path;
			path.addPolygon(polygon);
			painter->setPen(pen);
			painter->drawPath(path);
			painter->fillPath(path, brush);
		}
		static void renderEdge(QPainter* painter, World* world, const ShapePrimitive& shape, const QPen& pen)
		{
			assert(painter != nullptr && world != nullptr);
			assert(shape.shape->type() == Shape::Type::Edge);
			Edge* edge = dynamic_cast<Edge*>(shape.shape);
			renderPoint(painter, world, edge->startPoint() + shape.transform, pen);
			renderPoint(painter, world, edge->endPoint() + shape.transform, pen);
			renderLine(painter, world, edge->startPoint() + shape.transform, edge->endPoint() + shape.transform, pen);
		}
		static void renderRectangle(QPainter* painter, World* world, const ShapePrimitive& shape, const QPen& pen)
		{
			assert(painter != nullptr && world != nullptr);
			assert(shape.shape->type() == Shape::Type::Polygon);
			renderPoint(painter, world, shape.transform, pen);
			
			Vector2 position = world->worldToScreen(shape.transform);
			QPolygonF polygon;

			for (const Vector2& point : dynamic_cast<Rectangle*>(shape.shape)->vertices())
			{
				const Vector2 world_p = Matrix2x2(shape.rotation).multiply(point) + shape.transform;
				const Vector2 screen_p = world->worldToScreen(world_p);
				polygon << QPointF(screen_p.x, screen_p.y);
			}

			QColor color = pen.color();
			color.setAlphaF(0.15f);
			QBrush brush(color);
			QPainterPath path;
			path.addPolygon(polygon);
			painter->setPen(pen);
			painter->drawPath(path);
			painter->fillPath(path, brush);
		}
		static void renderCircle(QPainter* painter, World* world, const ShapePrimitive& shape, const QPen& pen)
		{
			assert(painter != nullptr && world != nullptr);
			assert(shape.shape->type() == Shape::Type::Circle);
			const Circle* circle = dynamic_cast<Circle*>(shape.shape);
			const Vector2 screen_p = world->worldToScreen(shape.transform);
			renderPoint(painter, world, shape.transform, pen);

			QColor color = pen.color();
			color.setAlphaF(0.15f);
			QBrush brush(color);
			QPainterPath path;
			path.addEllipse(QRectF(-circle->radius(), -circle->radius(), 2 * circle->radius(), 2 * circle->radius()));
			//QPainter default rotation orientation is clockwise
			painter->translate(screen_p.x, screen_p.y);
			painter->rotate(-shape.rotation);
			painter->setPen(pen);
			painter->drawPath(path);
			painter->fillPath(path, brush);
			painter->rotate(shape.rotation);
			painter->translate(-screen_p.x, -screen_p.y);
			
		}
		static void renderEllipse(QPainter* painter, World* world, const ShapePrimitive& shape, const QPen& pen)
		{
			assert(painter != nullptr && world != nullptr);
			assert(shape.shape->type() == Shape::Type::Ellipse);
			renderPoint(painter, world, shape.transform, pen);
			const Ellipse* ellipse = dynamic_cast<Ellipse*>(shape.shape);
			const Vector2 screen_p = world->worldToScreen(shape.transform);
			real A = ellipse->A();
			real B = ellipse->B();
			
			QColor color = pen.color();
			color.setAlphaF(0.15f);
			QBrush brush(color);
			QPainterPath path;
			painter->translate(screen_p.x, screen_p.y);
			painter->rotate(-shape.rotation);
			path.addEllipse(QRectF(-A, -B, ellipse->width(), ellipse->height()));
			painter->setPen(pen);
			painter->drawPath(path);
			painter->fillPath(path, brush);
			painter->rotate(shape.rotation);
			painter->translate(-screen_p.x, -screen_p.y);
			
		}
		static void renderCurve(QPainter* painter, World* world, const ShapePrimitive& shape, const QPen& pen)
		{
			assert(painter != nullptr && world != nullptr);
			assert(shape.shape->type() == Shape::Type::Curve);
			const Curve* curve = dynamic_cast<Curve*>(shape.shape);

			renderPoint(painter, world, curve->startPoint() + shape.transform, pen);
			renderPoint(painter, world, curve->endPoint() + shape.transform, pen);
			
            const Vector2 screen_start = world->worldToScreen(curve->startPoint() + shape.transform);
            const Vector2 screen_end = world->worldToScreen(curve->endPoint() + shape.transform);
            const Vector2 screen_control1 = world->worldToScreen(curve->control1() + shape.transform);
            const Vector2 screen_control2 = world->worldToScreen(curve->control2() + shape.transform);
			
			painter->rotate(-shape.rotation);
            QPainterPath path;
            path.moveTo(QPointF(screen_start.x, screen_start.y));
            path.cubicTo(QPointF(screen_control1.x, screen_control1.y), QPointF(screen_control2.x, screen_control2.y),
                         QPointF(screen_end.x, screen_end.y));
            painter->setPen(pen);
            painter->drawPath(path);
			painter->rotate(shape.rotation);
		}
		static void renderAngleLine(QPainter* painter, World* world, const ShapePrimitive& shape, const QPen& pen)
		{
			assert(painter != nullptr && world != nullptr);
            switch (shape.shape->type()) {
                case Shape::Type::Circle:
                {

                    break;
                }
                case Shape::Type::Polygon:
                {

                    break;
                }
                case Shape::Type::Ellipse:
                {

                    break;
                }
                default:
                    break;
            }
		}
		static void renderAABB(QPainter *painter, World* world, const AABB& aabb, const QPen& pen)
		{
			assert(painter != nullptr && world != nullptr);
			painter->setPen(pen);
			Vector2 topLeft;
			topLeft.set(-aabb.width / 2, aabb.height / 2);
			topLeft += aabb.position;
			topLeft = world->worldToScreen(topLeft);
			painter->drawRect(QRectF(topLeft.x, topLeft.y, aabb.width, aabb.height));
		}
		static void renderShape(QPainter *painter, World *world, const ShapePrimitive& shape, const QPen& pen)
		{
			assert(painter != nullptr && world != nullptr);
			switch (shape.shape->type())
			{
			case Shape::Type::Polygon:
			{
				renderPolygon(painter, world, shape, pen);
				break;
			}
			case Shape::Type::Ellipse:
			{
				renderEllipse(painter, world, shape, pen);
				break;
			}
			case Shape::Type::Circle:
			{
				renderCircle(painter, world, shape, pen);
				break;
			}
			case Shape::Type::Curve:
			{
				renderCurve(painter, world, shape, pen);
				break;
			}
			case Shape::Type::Edge:
			{
				renderEdge(painter, world, shape, pen);
				break;
			}
			default: break;
			}
		}
	private:
		
	};
}
#endif
