#ifndef PHYSICS2D_RENDERER_IMPL_QT_H
#define PHYSICS2D_RENDERER_IMPL_QT_H
#include "QPainter"
#include "QPointF"
#include "QPainterPath"
#include "QPen"
#include "QBrush"
#include "include/utils/camera.h"

namespace Physics2D
{
	class RendererQtImpl
	{
	public:
		static void renderPoint(QPainter* painter, Utils::Camera* camera, const Vector2& point, const QPen& pen)
		{
			assert(painter != nullptr && camera != nullptr);
			Vector2 screen_p = camera->worldToScreen(point);
			QPen p = pen;
			p.setWidth(4);
			painter->setPen(p);
			painter->drawPoint(QPointF(screen_p.x, screen_p.y));
		}
		static void renderLine(QPainter* painter, Utils::Camera* camera, const Vector2& p1, const Vector2& p2, const QPen& pen)
		{
			assert(painter != nullptr && camera != nullptr);
			Vector2 screen_p1 = camera->worldToScreen(p1);
			Vector2 screen_p2 = camera->worldToScreen(p2);
			painter->setPen(pen);
			painter->drawLine(QPointF(screen_p1.x, screen_p1.y), QPointF(screen_p2.x, screen_p2.y));
		}
		static void renderPolygon(QPainter* painter, Utils::Camera* camera, const ShapePrimitive& shape, const QPen& pen)
		{
			assert(painter != nullptr && camera != nullptr);
			assert(shape.shape->type() == Shape::Type::Polygon);
			Vector2 position = camera->worldToScreen(shape.transform);
			renderPoint(painter, camera, shape.transform, pen);
			QPen center(Qt::gray, 8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
			renderPoint(painter, camera, Matrix2x2(shape.rotation).multiply(dynamic_cast<Polygon*>(shape.shape.get())->center()) + shape.transform, center);
			QPolygonF polygon;
			QColor color = pen.color();
			color.setAlphaF(0.2);
			QBrush brush(color);

			for(const Vector2& point: dynamic_cast<Polygon*>(shape.shape.get())->vertices())
			{
				const Vector2 world_p = Matrix2x2(shape.rotation).multiply(point) + shape.transform;
				const Vector2 screen_p = camera->worldToScreen(world_p);
				polygon << QPointF(screen_p.x, screen_p.y);
			}
			QPainterPath path;
			path.addPolygon(polygon);
			painter->setPen(pen);
			painter->drawPath(path);
			painter->fillPath(path, brush);

			center.setColor(pen.color());
			center.setWidth(2);
			renderAngleLine(painter, camera, shape, center);
		}
		static void renderEdge(QPainter* painter, Utils::Camera* camera, const ShapePrimitive& shape, const QPen& pen)
		{
			assert(painter != nullptr && camera != nullptr);
			assert(shape.shape->type() == Shape::Type::Edge);
			Edge* edge = dynamic_cast<Edge*>(shape.shape.get());
			renderPoint(painter, camera, edge->startPoint() + shape.transform, pen);
			renderPoint(painter, camera, edge->endPoint() + shape.transform, pen);
			renderLine(painter, camera, edge->startPoint() + shape.transform, edge->endPoint() + shape.transform, pen);
		}
		static void renderRectangle(QPainter* painter, Utils::Camera* camera, const ShapePrimitive& shape, const QPen& pen)
		{
			assert(painter != nullptr && camera != nullptr);
			assert(shape.shape->type() == Shape::Type::Polygon);
			renderPoint(painter, camera, shape.transform, pen);
			
			Vector2 position = camera->worldToScreen(shape.transform);
			QPolygonF polygon;

			for (const Vector2& point : dynamic_cast<Rectangle*>(shape.shape.get())->vertices())
			{
				const Vector2 world_p = Matrix2x2(shape.rotation).multiply(point) + shape.transform;
				const Vector2 screen_p = camera->worldToScreen(world_p);
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
			renderAngleLine(painter, camera, shape, center);
		}
		static void renderCircle(QPainter* painter, Utils::Camera* camera, const ShapePrimitive& shape, const QPen& pen)
		{
			assert(painter != nullptr && camera != nullptr);
			assert(shape.shape->type() == Shape::Type::Circle);
			const Circle* circle = dynamic_cast<Circle*>(shape.shape.get());
			const Vector2 screen_p = camera->worldToScreen(shape.transform);
			renderPoint(painter, camera, shape.transform, pen);

			QColor color = pen.color();
			color.setAlphaF(0.15f);
			QBrush brush(color);
			QPainterPath path;
			const real radius = circle->radius() * camera->meterToPixel();
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
			renderAngleLine(painter, camera, shape, center);
		}

		static void renderCapsule(QPainter* painter, Utils::Camera* camera, const ShapePrimitive& shape, const QPen& pen)
		{
			assert(painter != nullptr && camera != nullptr);
			assert(shape.shape->type() == Shape::Type::Capsule);
			const Capsule* capsule = dynamic_cast<Capsule*>(shape.shape.get());
			const Vector2 screen_p = camera->worldToScreen(shape.transform);
			renderPoint(painter, camera, shape.transform, pen);

			QColor color = pen.color();
			color.setAlphaF(0.15f);
			QBrush brush(color);
			QPainterPath path;
			QRectF rect(-capsule->width() / 2 * camera->meterToPixel(), -capsule->height() / 2 * camera->meterToPixel(), capsule->width() * camera->meterToPixel(), capsule->height() * camera->meterToPixel());
			real shortest = Math::min(capsule->width(), capsule->height()) * camera->meterToPixel() / 2;
			path.addRoundedRect(rect, shortest, shortest);
			painter->translate(screen_p.x, screen_p.y);
			
			painter->rotate(Math::radianToAngle(-shape.rotation));

			painter->setPen(pen);
			painter->drawPath(path);
			painter->fillPath(path, brush);
			painter->rotate(Math::radianToAngle(shape.rotation));
			painter->translate(-screen_p.x, -screen_p.y);


			QPen center = pen;
			center.setWidth(2);
			renderAngleLine(painter, camera, shape, center);
		}
		static void renderEllipse(QPainter* painter, Utils::Camera* camera, const ShapePrimitive& shape, const QPen& pen)
		{
			assert(painter != nullptr && camera != nullptr);
			assert(shape.shape->type() == Shape::Type::Ellipse);
			renderPoint(painter, camera, shape.transform, pen);
			const Ellipse* ellipse = dynamic_cast<Ellipse*>(shape.shape.get());
			const Vector2 screen_p = camera->worldToScreen(shape.transform);
			real A = ellipse->A() * camera->meterToPixel();
			real B = ellipse->B() * camera->meterToPixel();
			real width = ellipse->width() * camera->meterToPixel();
			real height = ellipse->height() * camera->meterToPixel();
			QColor color = pen.color();
			color.setAlphaF(0.15f);
			QBrush brush(color);
			QPainterPath path;

			QPen center = pen;
			center.setWidth(2);
			renderAngleLine(painter, camera, shape, center);
			
			painter->translate(screen_p.x, screen_p.y);
			painter->rotate(-shape.rotation);
			path.addEllipse(QRectF(-A, -B, width, height));
			painter->setPen(pen);
			painter->drawPath(path);
			painter->fillPath(path, brush);
			painter->rotate(shape.rotation);
			painter->translate(-screen_p.x, -screen_p.y);

			
		}
		static void renderCurve(QPainter* painter, Utils::Camera* camera, const ShapePrimitive& shape, const QPen& pen)
		{
			assert(painter != nullptr && camera != nullptr);
			assert(shape.shape->type() == Shape::Type::Curve);
			const Curve* curve = dynamic_cast<Curve*>(shape.shape.get());

			renderPoint(painter, camera, curve->startPoint() + shape.transform, pen);
			renderPoint(painter, camera, curve->endPoint() + shape.transform, pen);
			
            const Vector2 screen_start = camera->worldToScreen(curve->startPoint() + shape.transform);
            const Vector2 screen_end = camera->worldToScreen(curve->endPoint() + shape.transform);
            const Vector2 screen_control1 = camera->worldToScreen(curve->control1() + shape.transform);
            const Vector2 screen_control2 = camera->worldToScreen(curve->control2() + shape.transform);
			
			painter->rotate(-shape.rotation);
            QPainterPath path;
            path.moveTo(QPointF(screen_start.x, screen_start.y));
            path.cubicTo(QPointF(screen_control1.x, screen_control1.y), QPointF(screen_control2.x, screen_control2.y),
                         QPointF(screen_end.x, screen_end.y));
            painter->setPen(pen);
            painter->drawPath(path);
			painter->rotate(shape.rotation);
		}
		static void renderAngleLine(QPainter* painter, Utils::Camera* camera, const ShapePrimitive& shape, const QPen& pen)
		{
			assert(painter != nullptr && camera != nullptr);
			QColor colorX("#8BC34A");
			QColor colorY("#FFEB3B");
			QPen xAxis(colorX, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
			QPen yAxis(colorY, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
			Vector2 xP(0.1, 0);
			Vector2 yP(0, 0.1);
			xP = Matrix2x2(shape.rotation).multiply(xP) + shape.transform;
			yP = Matrix2x2(shape.rotation).multiply(yP) + shape.transform;
			renderLine(painter, camera, shape.transform, xP, xAxis);
			renderLine(painter, camera, shape.transform, yP, yAxis);
		}
		static void renderAABB(QPainter *painter, Utils::Camera* camera, const AABB& aabb, const QPen& pen)
		{
			assert(painter != nullptr && camera != nullptr);
			painter->setPen(pen);
			Vector2 topLeft;
			topLeft.set(-aabb.width * (0.5), aabb.height * (0.5));
			topLeft += aabb.position;
			topLeft = camera->worldToScreen(topLeft);
			painter->drawRect(QRectF(topLeft.x, topLeft.y, aabb.width * camera->meterToPixel(), aabb.height * camera->meterToPixel()));
		}
		static void renderAngleJoint(QPainter* painter, Utils::Camera* camera, Joint* joint, const QPen& pen)
		{
			assert(joint != nullptr);
			AngleJoint* angleJoint = dynamic_cast<AngleJoint*>(joint);
		}
		static void renderDistanceJoint(QPainter* painter, Utils::Camera* camera, Joint* joint, const QPen& pen)
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
			renderPoint(painter, camera, pa, p);
			renderPoint(painter, camera, pb, p);
			renderPoint(painter, camera, minPoint, min);
			renderPoint(painter, camera, maxPoint, max);
			QColor color = Qt::green;
			color.setAlphaF(0.45);
			QPen line(color, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
			renderLine(painter, camera, pa, pb, line);

		}
		static void renderMouseJoint(QPainter* painter, Utils::Camera* camera, Joint* joint, const QPen& pen)
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
			renderPoint(painter, camera, pa, min);
			renderPoint(painter, camera, pb, max);
			QColor color = Qt::green;
			color.setAlphaF(0.45);
			QPen line(color, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
			renderLine(painter, camera, pa, pb, line);

		}
		static void renderPointJoint(QPainter* painter, Utils::Camera* camera, Joint* joint, const QPen& pen)
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
			renderPoint(painter, camera, pa, p);
			renderPoint(painter, camera, pb, p);
			renderLine(painter, camera, pa, pb, line);
		}
		static void renderPulleyJoint(QPainter* painter, Utils::Camera* camera, Joint* joint, const QPen& pen)
		{
			assert(joint != nullptr);

		}
		static void renderPrismaticJoint(QPainter* painter, Utils::Camera* camera, Joint* joint, const QPen& pen)
		{
			assert(joint != nullptr);

		}
		static void renderRevoluteJoint(QPainter* painter, Utils::Camera* camera, Joint* joint, const QPen& pen)
		{
			assert(joint != nullptr);

		}
		static void renderWheelJoint(QPainter* painter, Utils::Camera* camera, Joint* joint, const QPen& pen)
		{
			assert(joint != nullptr);

		}
		static void renderJoint(QPainter* painter, Utils::Camera* camera, Joint* joint, const QPen& pen)
		{
			assert(painter != nullptr && camera != nullptr);

			switch (joint->type())
			{
			case JointType::Angle:
			{
				renderAngleJoint(painter, camera, joint, pen);
				break;
			}
			case JointType::Distance:
			{
				renderDistanceJoint(painter, camera, joint, pen);
				break;
			}
			case JointType::Mouse:
			{
				renderMouseJoint(painter, camera, joint, pen);
				break;
			}
			case JointType::Point:
			{
				renderPointJoint(painter, camera, joint, pen);
				break;
			}
			case JointType::Pulley:
			{
				renderPulleyJoint(painter, camera, joint, pen);
				break;
			}
			case JointType::Prismatic:
			{
				renderPrismaticJoint(painter, camera, joint, pen);
				break;
			}
			case JointType::Revolute:
			{
				renderRevoluteJoint(painter, camera, joint, pen);
				break;
			}
			case JointType::Wheel:
			{
				renderWheelJoint(painter, camera, joint, pen);
				break;
			}
			default:
				break;
			}
		}
		static void renderShape(QPainter *painter, Utils::Camera* camera, const ShapePrimitive& shape, const QPen& pen)
		{
			assert(painter != nullptr && camera != nullptr);
			switch (shape.shape->type())
			{
			case Shape::Type::Polygon:
			{
				renderPolygon(painter, camera, shape, pen);
				break;
			}
			case Shape::Type::Ellipse:
			{
				renderEllipse(painter, camera, shape, pen);
				break;
			}
			case Shape::Type::Circle:
			{
				renderCircle(painter, camera, shape, pen);
				break;
			}
			case Shape::Type::Curve:
			{
				renderCurve(painter, camera, shape, pen);
				break;
			}
			case Shape::Type::Edge:
			{
				renderEdge(painter, camera, shape, pen);
				break;
			}
			case Shape::Type::Capsule:
			{
				renderCapsule(painter, camera, shape, pen);
				break;
			}
			default:
				break;
			}
		}
		
	};
}
#endif
