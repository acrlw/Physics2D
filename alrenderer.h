#ifndef ALRENDERER_H
#define ALRENDERER_H

#include <QObject>
#include <QPainter>
#include <QMouseEvent>
#include <QVector>
#include <QDebug>
#include <QPainterPath>
#include <albody.h>
class alRenderer : public QObject
{
    Q_OBJECT
public:
    alRenderer(const bool visible = true , const QColor &centerColor = Qt::darkBlue);
    enum RenderType{
        Measurer,
        Circle,
        Polygon,
        Wall,
        World,
        Ellipse,
        Segment,
        Curve
    };


    bool visible() const
    {
        return m_visible;
    }

    void setVisible(bool visible)
    {
        m_visible = visible;
    }
    qreal angleLineThickness() const
    {
        return m_angleLineThickness;
    }

    void setAngleLineThickness(const qreal &angleLineThickness)
    {
        m_angleLineThickness = angleLineThickness;
    }


    RenderType type() const
    {
        return m_type;
    }

    void setType(const RenderType &type)
    {
        m_type = type;
    }

    QPen& strokePen()
    {
        return m_strokePen;
    }

    void setStrokePen(const QPen &strokePen)
    {
        m_strokePen = strokePen;
    }


    QBrush& fillBrush()
    {
        return m_fillBrush;
    }

    void setFillBrush(const QBrush &fillBrush)
    {
        m_fillBrush = fillBrush;
    }

    qreal thickness() const
    {
        return m_thickness;
    }

    void setThickness(const qreal &thickness)
    {
        m_thickness = thickness;
    }



public slots:
    void handleMousePressEvent(QMouseEvent *e);
    void handleMouseMoveEvent(QMouseEvent *e);
    void handleMouseReleaseEvent(QMouseEvent *e);
    void handlePaintEvent(QPainter *e);
    virtual void render(QPainter *e) = 0;
protected:

    bool m_visible;
    qreal m_angleLineThickness;
    QColor m_centerColor;
    QBrush m_fillBrush;
    QPen m_strokePen;
    QPen b;
    qreal m_thickness;
    RenderType m_type;
};
class alBodyRenderer : public alRenderer
{
    Q_OBJECT
public:
    alBodyRenderer() {
        m_touchPen  = QPen(QColor(252, 3, 4), m_thickness, Qt::SolidLine, Qt::RoundCap);
    };

    void renderPositionCenter(QPainter * e, alBody *body, const QColor &color);

    void renderMassCenter(QPainter * e, alBody *body, const QColor &color = Qt::darkGray);
protected:
    QPen m_touchPen;
};
class alMeasurer : public alRenderer
{
    Q_OBJECT
public:
    alMeasurer(const qreal arrowSize = 8, const qreal fontSize = 10, const qreal distance = 20, const qreal thickness = 2, const QColor &accelerationColor = Qt::darkRed, const QColor &velocityColor = Qt::darkCyan);
    inline QVector<alBody *>& bodyList()
    {
        return m_bodyList;
    }
    inline void setBodyList(const QVector<alBody *> &bodyList)
    {
        m_bodyList = bodyList;
    }
    inline bool visible() const
    {
        return m_visible;
    }
    inline void setVisible(bool visible)
    {
        m_visible = visible;
    }

    qreal arrowSize() const
    {
        return m_arrowSize;
    }

    void setArrowSize(qreal arrowSize)
    {
        m_arrowSize = arrowSize;
    }
    qreal textDistance() const
    {
        return m_textDistance;
    }

    void setTextDistance(const qreal &textDistance)
    {
        m_textDistance = textDistance;
    }

    qreal fontSize() const
    {
        return m_fontSize;
    }

    void setFontSize(const qreal &fontSize)
    {
        m_fontSize = fontSize;
    }
    QColor velocityColor() const
    {
        return m_velocityColor;
    }

    void setVelocityColor(const QColor &velocityColor)
    {
        m_velocityColor = velocityColor;
    }

    QColor accelerationColor() const
    {
        return m_accelerationColor;
    }

    void setAccelerationColor(const QColor &accelerationColor)
    {
        m_accelerationColor = accelerationColor;
    }

    void renderArrow(QPainter *painter, const QPointF &start, const QPointF &end, const QString &text, const QColor& color);

public slots:

    void render(QPainter *e)override;
private:
    QColor m_accelerationColor;
    QColor m_velocityColor;
    QPainter * m_painter;
    QVector<alBody *> m_bodyList;
    bool m_centerPathVisible;
    bool m_visible;
    qreal m_arrowSize;
    qreal m_fontSize;
    qreal m_textDistance;
};

class alCircleRenderer : public alBodyRenderer
{
    Q_OBJECT
public:
    alCircleRenderer(const qreal angleLineThickness = 1);


    inline bool isInArea(alCircle * circle, const QPointF& pos)
    {
        return alVector2(pos.x() - circle->position().x(), pos.y() - circle->position().y()).lengthSquare() <= circle->radius() * circle->radius();
    }


    QVector<alCircle *>& circleList()
    {
        return m_circleList;
    }

    void setCircleList(const QVector<alCircle *> &circleList)
    {
        m_circleList = circleList;
    }



public slots:
    void render(QPainter *e)override;
private:
    QVector<alCircle *> m_circleList;
};

class alPolygonRenderer : public alBodyRenderer
{
    Q_OBJECT
public:
    alPolygonRenderer(){

    }


    QVector<alPolygon *>& polygonList()
    {
        return m_polygonList;
    }

    void setPolygonList(const QVector<alPolygon *> &polygonList)
    {
        m_polygonList = polygonList;
    }
    void renderMassCenter(QPainter *e, alBody *body, const QColor &color = Qt::darkGray);
    QPolygonF updateVertices(alPolygon* polygon)
    {
        QPolygonF vertex;
        foreach (alVector2 v, polygon->getRotatedVertices()) {
            vertex.prepend(QPointF(v.x(), v.y()) + QPointF(polygon->position().x(), polygon->position().y()));
        }
        return vertex;
    }
public slots:
    void render(QPainter *e)override;
private:
    QVector<alPolygon *> m_polygonList;
};

class alRectangleRenderer : public alPolygonRenderer
{
    Q_OBJECT
public:
    alRectangleRenderer(const qreal borderThickness = 1);

    //will deprecate
    inline bool isInArea(alRectangle * rectangle, const QPointF& pos)
    {
        return (abs(pos.x() - rectangle->position().x()) < rectangle->width() / 2) &&
                (abs(pos.y() - rectangle->position().y()) < rectangle->height() / 2);
    }
    QVector<alRectangle *>& rectangleList()
    {
        return m_rectangleList;
    }

    void setRectangleList(const QVector<alRectangle *> &rectangleList)
    {
        m_rectangleList = rectangleList;
    }


public slots:
    void render(QPainter *e)override;
protected:
    QPolygonF m_rectVertex;
    QVector<alRectangle *> m_rectangleList;
};
class alEllipseRenderer : public alBodyRenderer
{
    Q_OBJECT
public:
    alEllipseRenderer() {
        m_type = RenderType::Ellipse;
    }
    QVector<alEllipse *>& ellipseList() ;

public slots:
    void render(QPainter * e)override;
private:

    QVector<alEllipse *> m_ellipseList;
    void renderGeometricCenter(QPainter *e, alEllipse *body, const QColor &color = Qt::darkGray);
};
class alWallRenderer: public alRectangleRenderer{
    Q_OBJECT
public:
    alWallRenderer(){
        m_type = RenderType::Wall;
    }
    QVector<alWall*> &wallList()
    {
        return m_wallList;
    }
    void setWallList(const QVector<alWall*> &wallList)
    {
        m_wallList = wallList;
    }

public slots:
    void render(QPainter * e)override;
private:
    QVector<alWall*> m_wallList;
};

class alEdgeRenderer: public alRenderer
{
    Q_OBJECT
public:
    alEdgeRenderer(){
        m_type = RenderType::Segment;
    }
    QVector<alEdge*> &edgeList()
    {
        return m_edgeList;
    }

public slots:
    void render(QPainter * e)override;
private:
    QVector<alEdge*> m_edgeList;
};

class alCurveRenderer: public alRenderer
{
    Q_OBJECT
public:
    alCurveRenderer() {
        m_type = RenderType::Curve;
    }

    QVector<alCurveEdge*> &edgeList()
    {
        return m_edgeList;
    }

public slots:
    void render(QPainter * e)override;
private:
    QVector<alCurveEdge*> m_edgeList;
};
class alWorldRenderer: public alRenderer
{
public:
    alWorldRenderer() {
        m_type = RenderType::World;
    }
};
#endif // ALRENDERER_H
