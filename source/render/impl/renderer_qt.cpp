#include "include/render/impl/renderer_qt.h"

namespace Physics2D
{

    void RendererQtImpl::renderPoint(QPainter* painter, Utils::Camera* camera, const Vector2& point, const QPen& pen)
    {
        assert(painter != nullptr && camera != nullptr);
        Vector2 screen_p = camera->worldToScreen(point);
        QPen p = pen;
        p.setWidth(4);
        painter->setPen(p);
        painter->drawPoint(QPointF(screen_p.x, screen_p.y));
    }

    void RendererQtImpl::renderLine(QPainter* painter, Utils::Camera* camera, const Vector2& p1, const Vector2& p2, const QPen& pen)
    {
        assert(painter != nullptr && camera != nullptr);
        Vector2 screen_p1 = camera->worldToScreen(p1);
        Vector2 screen_p2 = camera->worldToScreen(p2);
        painter->setPen(pen);
        painter->drawLine(QPointF(screen_p1.x, screen_p1.y), QPointF(screen_p2.x, screen_p2.y));
    }

    void RendererQtImpl::renderPoints(QPainter* painter, Utils::Camera* camera, const std::vector<Vector2>& points,
	    const QPen& pen)
    {
        assert(painter != nullptr && camera != nullptr);
        QPolygonF pts;
        for(auto& elem: points)
        {
            Vector2 screen_p = camera->worldToScreen(elem);
            pts << QPointF(screen_p.x, screen_p.y);
        }
        QPen p = pen;
        p.setWidth(4);
        painter->setPen(p);
        painter->drawPoints(pts);
    }

    void RendererQtImpl::renderLines(QPainter* painter, Utils::Camera* camera,
	    const std::vector<std::pair<Vector2, Vector2>>& lines, const QPen& pen)
    {
        assert(painter != nullptr && camera != nullptr);
        QVector<QLineF> lfs;
        for(auto& [p1, p2] : lines)
        {
            Vector2 screen_p1 = camera->worldToScreen(p1);
            Vector2 screen_p2 = camera->worldToScreen(p2);
            lfs << QLineF(QPointF(screen_p1.x, screen_p1.y), QPointF(screen_p2.x, screen_p2.y));
        }
        painter->setPen(pen);
        painter->drawLines(lfs);
    }

    void RendererQtImpl::renderPolygon(QPainter* painter, Utils::Camera* camera, const ShapePrimitive& shape, const QPen& pen)
    {
        assert(painter != nullptr && camera != nullptr);
        assert(shape.shape->type() == Shape::Type::Polygon);
        Vector2 position = camera->worldToScreen(shape.transform);
        QPen center(Qt::gray, 8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        renderPoint(painter, camera, shape.transform, center);
        QPolygonF polygon;
        QColor color = pen.color();
        color.setAlphaF(0.2);
        QBrush brush(color);

        for (const Vector2& point : dynamic_cast<Polygon*>(shape.shape.get())->vertices())
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

    void RendererQtImpl::renderEdge(QPainter* painter, Utils::Camera* camera, const ShapePrimitive& shape, const QPen& pen)
    {
        assert(painter != nullptr && camera != nullptr);
        assert(shape.shape->type() == Shape::Type::Edge);
        Edge* edge = dynamic_cast<Edge*>(shape.shape.get());
        renderPoint(painter, camera, edge->startPoint() + shape.transform, pen);
        renderPoint(painter, camera, edge->endPoint() + shape.transform, pen);
        renderLine(painter, camera, edge->startPoint() + shape.transform, edge->endPoint() + shape.transform, pen);
        QColor colorY("#FFEB3B");
        QPen yAxis(colorY, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        Vector2 center = (edge->startPoint() + edge->endPoint()) / 2.0;
        center += shape.transform;
        renderLine(painter, camera, center, center + 0.1 * edge->normal(), yAxis);
    }

    void RendererQtImpl::renderRectangle(QPainter* painter, Utils::Camera* camera, const ShapePrimitive& shape, const QPen& pen)
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

    void RendererQtImpl::renderCircle(QPainter* painter, Utils::Camera* camera, const ShapePrimitive& shape, const QPen& pen)
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

    void RendererQtImpl::renderCapsule(QPainter* painter, Utils::Camera* camera, const ShapePrimitive& shape, const QPen& pen)
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

        painter->rotate(Math::radianToDegree(-shape.rotation));

        painter->setPen(pen);
        painter->drawPath(path);
        painter->fillPath(path, brush);
        painter->rotate(Math::radianToDegree(shape.rotation));
        painter->translate(-screen_p.x, -screen_p.y);


        QPen center = pen;
        center.setWidth(2);
        renderAngleLine(painter, camera, shape, center);
    }

    void RendererQtImpl::renderSector(QPainter* painter, Utils::Camera* camera, const ShapePrimitive& shape, const QPen& pen)
    {
        assert(painter != nullptr && camera != nullptr);
        assert(shape.shape->type() == Shape::Type::Sector);
        const Sector* sector = dynamic_cast<Sector*>(shape.shape.get());
        
        Vector2 offset(sector->radius(), sector->radius());
        Vector2 topLeft = camera->worldToScreen(shape.transform - offset);
        Vector2 bottomRight = camera->worldToScreen(shape.transform + offset);
        QPen gc(Qt::gray, 8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        renderPoint(painter, camera, Matrix2x2(shape.rotation).multiply(dynamic_cast<Sector*>(shape.shape.get())->center()) + shape.transform, gc);

        QColor color = pen.color();
        color.setAlphaF(0.15f);
        QBrush brush(color);
        QBrush oldBrush = painter->brush();

        painter->setPen(pen);
        painter->setBrush(brush);

        
        QRectF boundingRect;
        boundingRect.setTopLeft({ topLeft.x, topLeft.y});
        boundingRect.setBottomRight({ bottomRight.x, bottomRight.y });

        auto start = Math::radianToDegree(sector->startRadian() + shape.rotation) * 16.0;
        auto span = Math::radianToDegree(sector->spanRadian()) * 16.0;
        painter->drawPie(boundingRect, start, span);
        

        QPen center = pen;
        center.setWidth(2);
        renderAngleLine(painter, camera, shape, center);
        painter->setBrush(oldBrush);
    }

    void RendererQtImpl::renderEllipse(QPainter* painter, Utils::Camera* camera, const ShapePrimitive& shape, const QPen& pen)
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
        painter->rotate(Math::radianToDegree(-shape.rotation));
        path.addEllipse(QRectF(-A, -B, width, height));
        painter->setPen(pen);
        painter->drawPath(path);
        painter->fillPath(path, brush);
        painter->rotate(Math::radianToDegree(shape.rotation));
        painter->translate(-screen_p.x, -screen_p.y);


    }

    void RendererQtImpl::renderCurve(QPainter* painter, Utils::Camera* camera, const ShapePrimitive& shape, const QPen& pen)
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

    void RendererQtImpl::renderAngleLine(QPainter* painter, Utils::Camera* camera, const ShapePrimitive& shape, const QPen& pen)
    {
        assert(painter != nullptr && camera != nullptr);
        QColor colorX("#8BC34A");
        QColor colorY("#FFEB3B");
        QPen xAxis(colorX, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        QPen yAxis(colorY, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        Vector2 xP(0.1, 0);
        Vector2 yP(0, 0.1);
        Vector2 mc = Matrix2x2(shape.rotation).multiply(shape.shape->center());
        xP = Matrix2x2(shape.rotation).multiply(xP) + shape.transform + mc;
        yP = Matrix2x2(shape.rotation).multiply(yP) + shape.transform + mc;
        renderLine(painter, camera, shape.transform + mc, xP, xAxis);
        renderLine(painter, camera, shape.transform + mc, yP, yAxis);
    }

    void RendererQtImpl::renderAABB(QPainter* painter, Utils::Camera* camera, const AABB& aabb, const QPen& pen)
    {
        assert(painter != nullptr && camera != nullptr);
        painter->setPen(pen);
        Vector2 topLeft = aabb.topLeft();
        topLeft = camera->worldToScreen(topLeft);
        painter->drawRect(QRectF(topLeft.x, topLeft.y, aabb.width * camera->meterToPixel(), aabb.height * camera->meterToPixel()));
    }

    void RendererQtImpl::renderRotationJoint(QPainter* painter, Utils::Camera* camera, Joint* joint, const QPen& pen)
    {
        assert(joint != nullptr);
        RotationJoint* angleJoint = dynamic_cast<RotationJoint*>(joint);
    }

    void RendererQtImpl::renderDistanceJoint(QPainter* painter, Utils::Camera* camera, Joint* joint, const QPen& pen)
    {
        assert(joint != nullptr);
        DistanceJoint* distanceJoint = dynamic_cast<DistanceJoint*>(joint);
        Vector2 pa = distanceJoint->primitive().bodyA->toWorldPoint(distanceJoint->primitive().localPointA);
        Vector2 pb = distanceJoint->primitive().targetPoint;
        Vector2 n = (pa - pb).normal();
        Vector2 minPoint = n * distanceJoint->primitive().minDistance + pb;
        Vector2 maxPoint = n * distanceJoint->primitive().maxDistance + pb;
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
        QColor color = Qt::gray;
        color.setAlphaF(0.45);
        QPen line(color, 1, Qt::DashDotLine, Qt::RoundCap, Qt::RoundJoin);
        renderLine(painter, camera, pa, pb, line);

    }
    

    void RendererQtImpl::renderPointJoint(QPainter* painter, Utils::Camera* camera, Joint* joint, const QPen& pen)
    {
        assert(joint != nullptr);
        PointJoint* pointJoint = dynamic_cast<PointJoint*>(joint);
        Vector2 pa = pointJoint->primitive().bodyA->toWorldPoint(pointJoint->primitive().localPointA);
        Vector2 pb = pointJoint->primitive().targetPoint;

        QColor pColor = Qt::gray;
        QPen p(pColor, 8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);

        QColor color = Qt::green;
        color.setAlphaF(0.45);
        QPen line(color, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        renderPoint(painter, camera, pa, p);
        renderPoint(painter, camera, pb, p);
        renderLine(painter, camera, pa, pb, line);
    }

    void RendererQtImpl::renderOrientationJoint(QPainter* painter, Utils::Camera* camera, Joint* joint, const QPen& pen)
    {
        assert(joint != nullptr);
        OrientationJoint* pointJoint = dynamic_cast<OrientationJoint*>(joint);
        Vector2 pa = pointJoint->primitive().bodyA->position();
        Vector2 pb = pointJoint->primitive().targetPoint;

        QColor pColor = Qt::gray;
        QPen p(pColor, 8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);

        QColor color = Qt::green;
        color.setAlphaF(0.45);
        QPen line(color, 1, Qt::DotLine, Qt::RoundCap, Qt::RoundJoin);
        renderPoint(painter, camera, pa, p);
        renderPoint(painter, camera, pb, p);
        renderLine(painter, camera, pa, pb, line);
    }

    void RendererQtImpl::renderPulleyJoint(QPainter* painter, Utils::Camera* camera, Joint* joint, const QPen& pen)
    {
        assert(joint != nullptr);

    }

    void RendererQtImpl::renderPrismaticJoint(QPainter* painter, Utils::Camera* camera, Joint* joint, const QPen& pen)
    {
        assert(joint != nullptr);

    }

    void RendererQtImpl::renderRevoluteJoint(QPainter* painter, Utils::Camera* camera, Joint* joint, const QPen& pen)
    {
        assert(joint != nullptr);

    }

    void RendererQtImpl::renderWheelJoint(QPainter* painter, Utils::Camera* camera, Joint* joint, const QPen& pen)
    {
        assert(joint != nullptr);

    }

    void RendererQtImpl::renderJoint(QPainter* painter, Utils::Camera* camera, Joint* joint, const QPen& pen)
    {
        assert(painter != nullptr && camera != nullptr);

        switch (joint->type())
        {
        case JointType::Rotation:
        {
            renderRotationJoint(painter, camera, joint, pen);
            break;
        }
        case JointType::Distance:
        {
            renderDistanceJoint(painter, camera, joint, pen);
            break;
        }
        case JointType::Point:
        {
            renderPointJoint(painter, camera, joint, pen);
            break;
        }
        case JointType::Orientation:
        {
            renderOrientationJoint(painter, camera, joint, pen);
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

    void RendererQtImpl::renderShape(QPainter* painter, Utils::Camera* camera, const ShapePrimitive& shape, const QPen& pen)
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
        case Shape::Type::Sector:
        {
            renderSector(painter, camera, shape, pen);
            break;
        }
        default:
            break;
        }
    }

}
