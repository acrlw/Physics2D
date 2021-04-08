#include "testbed/window.h"
namespace Physics2D
{

    Window::Window(QWidget *parent)
    {
        this->setParent(parent);
        this->setWindowTitle("Testbed");
        this->resize(1920, 1080);
        m_world.setGeometry({ 0,0 }, { 1920,1080 });
        m_rectShape.set(50, 50);
        m_polygon.append({{-4, 1},{-3, -5}, {5, -6}, {7, 4}, {0, 6}, {-4, 1}});
        m_polygon.scale(5);
        m_ellipse.set({ -20, 15 }, { 20, -15 });
        m_ellipse.scale(5);
        m_circle.setRadius(6);
        m_circle.scale(8);
        m_edge.set({-20, 40}, {40, 80});
        m_curve.set({-300, 200}, {-50, 20}, {0, 40}, {200, 150});
    }

    Window::~Window()
    {
    	
    }
    void Window::paintEvent(QPaintEvent *)
    {
        QPainter painter(this);
    	//prepare for background and clipping boundary
        painter.setRenderHint(QPainter::Antialiasing);
        painter.setClipRect(m_world.leftTop().x, m_world.leftTop().y, m_world.width(), m_world.height());
        painter.fillRect(QRectF(m_world.leftTop().x, m_world.leftTop().y, m_world.width(), m_world.height()), QBrush(QColor(51, 51, 51)));

        this->testShape(&painter);
    }

    void Window::resizeEvent(QResizeEvent *e)
    {
        m_world.setRightBottom(Vector2(e->size().width() - m_world.leftTop().x, e->size().height() - m_world.leftTop().y));
        this->repaint();
    }

    void Window::mousePressEvent(QMouseEvent *)
    {

    }

    void Window::mouseReleaseEvent(QMouseEvent *e)
    {

    }
     
    void Window::mouseMoveEvent(QMouseEvent *e)
    {

    }

    void Window::keyPressEvent(QKeyEvent *event)
    {

    }

    void Window::keyReleaseEvent(QKeyEvent *event)
    {

    }

    void Window::testShape(QPainter* painter)
    {

        ShapePrimitive shape, polygon, circle, ellipse, edge, curve;

        QPen pen(Qt::green, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);

        shape.shape = &m_rectShape;
        shape.rotation = -37;
        shape.transform.set(180, 120);
        RendererQtImpl::renderRectangle(painter, &m_world, shape, pen);
        pen.setWidth(10);
        RendererQtImpl::renderPoint(painter, &m_world, Vector2(0, 0), pen);

        pen.setWidth(2);
        polygon.shape = &m_polygon;
        polygon.transform.set(-50, 100);
        RendererQtImpl::renderPolygon(painter, &m_world, polygon, pen);

        circle.shape = &m_circle;
        circle.transform.set(100, 150);
        RendererQtImpl::renderCircle(painter, &m_world, circle, pen);

        ellipse.shape = &m_ellipse;
        ellipse.rotation = 45;
        ellipse.transform.set(-300, 160);
        RendererQtImpl::renderEllipse(painter, &m_world, ellipse, pen);

        edge.shape = &m_edge;
        edge.transform.set(0, 400);
        RendererQtImpl::renderEdge(painter, &m_world, edge, pen);

        curve.shape = &m_curve;
        curve.transform.set(0, 300);
        RendererQtImpl::renderCurve(painter, &m_world, curve, pen);
    }

}
