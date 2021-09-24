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
        static void renderPoint(QPainter* painter, Utils::Camera* camera, const Vector2& point, const QPen& pen);
        static void renderLine(QPainter* painter, Utils::Camera* camera, const Vector2& p1, const Vector2& p2, const QPen& pen);

        static void renderShape(QPainter* painter, Utils::Camera* camera, const ShapePrimitive& shape, const QPen& pen);
        static void renderPolygon(QPainter* painter, Utils::Camera* camera, const ShapePrimitive& shape, const QPen& pen);
        static void renderEdge(QPainter* painter, Utils::Camera* camera, const ShapePrimitive& shape, const QPen& pen);
        static void renderRectangle(QPainter* painter, Utils::Camera* camera, const ShapePrimitive& shape, const QPen& pen);
        static void renderCircle(QPainter* painter, Utils::Camera* camera, const ShapePrimitive& shape, const QPen& pen);
        static void renderCapsule(QPainter* painter, Utils::Camera* camera, const ShapePrimitive& shape, const QPen& pen);
        static void renderSector(QPainter* painter, Utils::Camera* camera, const ShapePrimitive& shape, const QPen& pen);
        static void renderEllipse(QPainter* painter, Utils::Camera* camera, const ShapePrimitive& shape, const QPen& pen);
        static void renderCurve(QPainter* painter, Utils::Camera* camera, const ShapePrimitive& shape, const QPen& pen);
        static void renderAngleLine(QPainter* painter, Utils::Camera* camera, const ShapePrimitive& shape, const QPen& pen);

        static void renderAABB(QPainter *painter, Utils::Camera* camera, const AABB& aabb, const QPen& pen);

        static void renderJoint(QPainter* painter, Utils::Camera* camera, Joint* joint, const QPen& pen);
        static void renderRotationJoint(QPainter* painter, Utils::Camera* camera, Joint* joint, const QPen& pen);
        static void renderDistanceJoint(QPainter* painter, Utils::Camera* camera, Joint* joint, const QPen& pen);
        static void renderMouseJoint(QPainter* painter, Utils::Camera* camera, Joint* joint, const QPen& pen);
        static void renderPointJoint(QPainter* painter, Utils::Camera* camera, Joint* joint, const QPen& pen);

        static void renderOrientationJoint(QPainter* painter, Utils::Camera* camera, Joint* joint, const QPen& pen);
        static void renderPulleyJoint(QPainter* painter, Utils::Camera* camera, Joint* joint, const QPen& pen);
        static void renderPrismaticJoint(QPainter* painter, Utils::Camera* camera, Joint* joint, const QPen& pen);
        static void renderRevoluteJoint(QPainter* painter, Utils::Camera* camera, Joint* joint, const QPen& pen);
        static void renderWheelJoint(QPainter* painter, Utils::Camera* camera, Joint* joint, const QPen& pen);
		
	};
}
#endif
