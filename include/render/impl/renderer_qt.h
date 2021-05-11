#ifndef PHYSICS2D_RENDERER_IMPL_QT_H
#define PHYSICS2D_RENDERER_IMPL_QT_H
#include "include/dynamics/world.h"
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
			color.setAlphaF(0.2);
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

			center.setColor(pen.color());
			center.setWidth(2);
			renderAngleLine(painter, world, shape, center);
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
			
			QPen center = pen;
			center.setWidth(2);
			renderAngleLine(painter, world, shape, center);
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
			const real radius = circle->radius() * Constant::MeterToPixel;
			path.addEllipse(QRectF(-radius, -radius, 2 * radius, 2 * radius));
			//QPainter default rotation orientation is clockwise
			painter->translate(screen_p.x, screen_p.y);
			painter->rotate(-shape.rotation);
			painter->setPen(pen);
			painter->drawPath(path);
			painter->fillPath(path, brush);
			painter->rotate(shape.rotation);
			painter->translate(-screen_p.x, -screen_p.y);


			QPen center = pen;
			center.setWidth(2);
			renderAngleLine(painter, world, shape, center);
		}
		static void renderEllipse(QPainter* painter, World* world, const ShapePrimitive& shape, const QPen& pen)
		{
			assert(painter != nullptr && world != nullptr);
			assert(shape.shape->type() == Shape::Type::Ellipse);
			renderPoint(painter, world, shape.transform, pen);
			const Ellipse* ellipse = dynamic_cast<Ellipse*>(shape.shape);
			const Vector2 screen_p = world->worldToScreen(shape.transform);
			real A = ellipse->A() * Constant::MeterToPixel;
			real B = ellipse->B() * Constant::MeterToPixel;
			real width = ellipse->width() * Constant::MeterToPixel;
			real height = ellipse->height() * Constant::MeterToPixel;
			QColor color = pen.color();
			color.setAlphaF(0.15f);
			QBrush brush(color);
			QPainterPath path;

			QPen center = pen;
			center.setWidth(2);
			renderAngleLine(painter, world, shape, center);
			
			painter->translate(screen_p.x, screen_p.y);
			painter->rotate(-shape.rotation);
			path.addEllipse(QRectF(-A, -B, width, height));
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
			QColor colorX("#8BC34A");
			QColor colorY("#FFEB3B");
			QPen xAxis(colorX, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
			QPen yAxis(colorY, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
			Vector2 xP(0.5, 0);
			Vector2 yP(0, 0.5);
			xP = Matrix2x2(shape.rotation).multiply(xP) + shape.transform;
			yP = Matrix2x2(shape.rotation).multiply(yP) + shape.transform;
			renderLine(painter, world, shape.transform, xP, xAxis);
			renderLine(painter, world, shape.transform, yP, yAxis);
		}
		static void renderAABB(QPainter *painter, World* world, const AABB& aabb, const QPen& pen)
		{
			assert(painter != nullptr && world != nullptr);
			painter->setPen(pen);
			Vector2 topLeft;
			topLeft.set(-aabb.width * (0.5), aabb.height * (0.5));
			topLeft += aabb.position;
			topLeft = world->worldToScreen(topLeft);
			painter->drawRect(QRectF(topLeft.x, topLeft.y, aabb.width * Constant::MeterToPixel, aabb.height * Constant::MeterToPixel));
		}
		static void renderAngleJoint(QPainter* painter, World* world, Joint* joint, const QPen& pen)
		{
			assert(joint != nullptr);
			AngleJoint* angleJoint = dynamic_cast<AngleJoint*>(joint);
		}
		static void renderDistanceJoint(QPainter* painter, World* world, Joint* joint, const QPen& pen)
		{
			assert(joint != nullptr);
			DistanceJoint* distanceJoint = dynamic_cast<DistanceJoint*>(joint);
			Vector2 pa = distanceJoint->primitive().bodyA->toWorldPoint(distanceJoint->primitive().localPointA);
			Vector2 pb = distanceJoint->primitive().bodyB->toWorldPoint(distanceJoint->primitive().localPointB);
			Vector2 n = (pb - pa).normal();
			Vector2 minPoint = n * distanceJoint->primitive().minDistance + pa;
			Vector2 maxPoint = n * distanceJoint->primitive().maxDistance + pa;
			QColor minColor("#448AFF");
			QColor maxColor("#F44336");
			minColor.setAlphaF(0.8);
			maxColor.setAlphaF(0.8);
			QPen min(minColor, 6, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
			QPen max(maxColor, 6, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
			QPen p(Qt::gray, 6, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
			renderPoint(painter, world, pa, p);
			renderPoint(painter, world, pb, p);
			renderPoint(painter, world, minPoint, min);
			renderPoint(painter, world, maxPoint, max);
			QColor color = Qt::green;
			color.setAlphaF(0.45);
			QPen line(color, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
			renderLine(painter, world, pa, pb, line);

		}
		static void renderMouseJoint(QPainter* painter, World* world, Joint* joint, const QPen& pen)
		{
			assert(joint != nullptr);

			MouseJoint* distanceJoint = dynamic_cast<MouseJoint*>(joint);
			Vector2 pa = distanceJoint->primitive().bodyA->toWorldPoint(distanceJoint->primitive().localPointA);
			Vector2 pb = distanceJoint->primitive().mousePoint;
			Vector2 n = (pb - pa).normal();
			QColor minColor("#448AFF");
			QColor maxColor("#F44336");
			minColor.setAlphaF(0.8);
			maxColor.setAlphaF(0.8);
			QPen min(minColor, 6, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
			QPen max(maxColor, 6, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
			renderPoint(painter, world, pa, min);
			renderPoint(painter, world, pb, max);
			QColor color = Qt::green;
			color.setAlphaF(0.45);
			QPen line(color, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
			renderLine(painter, world, pa, pb, line);

		}
		static void renderPointJoint(QPainter* painter, World* world, Joint* joint, const QPen& pen)
		{
			assert(joint != nullptr);
			PointJoint* pointJoint = dynamic_cast<PointJoint*>(joint);
			Vector2 pa = pointJoint->primitive().bodyA->toWorldPoint(pointJoint->primitive().localPointA);
			Vector2 pb = pointJoint->primitive().bodyB->toWorldPoint(pointJoint->primitive().localPointB);
			
			QColor pColor = Qt::gray;
			QPen p(pColor, 6, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
			
			QColor color = Qt::green;
			color.setAlphaF(0.45);
			QPen line(color, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
			renderPoint(painter, world, pa, p);
			renderPoint(painter, world, pb, p);
			renderLine(painter, world, pa, pb, line);
		}
		static void renderPulleyJoint(QPainter* painter, World* world, Joint* joint, const QPen& pen)
		{
			assert(joint != nullptr);

		}
		static void renderPrismaticJoint(QPainter* painter, World* world, Joint* joint, const QPen& pen)
		{
			assert(joint != nullptr);

		}
		static void renderRevoluteJoint(QPainter* painter, World* world, Joint* joint, const QPen& pen)
		{
			assert(joint != nullptr);

		}
		static void renderWheelJoint(QPainter* painter, World* world, Joint* joint, const QPen& pen)
		{
			assert(joint != nullptr);

		}
		static void renderJoint(QPainter* painter, World* world, Joint* joint, const QPen& pen)
		{
			assert(painter != nullptr && world != nullptr);

			switch (joint->type())
			{
			case JointType::Angle:
			{
				renderAngleJoint(painter, world, joint, pen);
				break;
			}
			case JointType::Distance:
			{
				renderDistanceJoint(painter, world, joint, pen);
				break;
			}
			case JointType::Mouse:
			{
				renderMouseJoint(painter, world, joint, pen);
				break;
			}
			case JointType::Point:
			{
				renderPointJoint(painter, world, joint, pen);
				break;
			}
			case JointType::Pulley:
			{
				renderPulleyJoint(painter, world, joint, pen);
				break;
			}
			case JointType::Prismatic:
			{
				renderPrismaticJoint(painter, world, joint, pen);
				break;
			}
			case JointType::Revolute:
			{
				renderRevoluteJoint(painter, world, joint, pen);
				break;
			}
			case JointType::Wheel:
			{
				renderWheelJoint(painter, world, joint, pen);
				break;
			}
			default:
				break;
			}
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
			default: 
				break;
			}
		}
		
	};
}
#endif
