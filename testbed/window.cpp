#include "testbed/window.h"
namespace Physics2D
{

    Window::Window(QWidget *parent)
    {
        this->setParent(parent);
        this->setWindowTitle("Testbed");
        this->resize(1920, 1080);
        this->setMouseTracking(true);
        m_world.setGeometry({ 0,0 }, { 1920,1080 });

    	
        rectangle.set(25, 25);
        rectangle.scale(2);

        
        polygon.append({ {-4, 1},{-3, -5}, {5, -6}, {7, 4}, {0, 6}, {-4, 1} });
        polygon.scale(15);
        ellipse.set({ -20, 15 }, { 20, -15 });
        ellipse.scale(5);
        circle.setRadius(6);
        circle.scale(12);
        edge.set({ -250, 40 }, { 350, 180 });
        curve.set({ -600, 200 }, { -50, 20 }, { 0, 40 }, { 600, 150 });
    	
        //createStackBox();
    }

    Window::~Window()
    {
    	
    }
    void Window::paintEvent(QPaintEvent *)
    {
        QPainter painter(this);
    	//prepare for background, origin and clipping boundary
        painter.setRenderHint(QPainter::Antialiasing);
        painter.setClipRect(m_world.leftTop().x, m_world.leftTop().y, m_world.width(), m_world.height());
        painter.fillRect(QRectF(m_world.leftTop().x, m_world.leftTop().y, m_world.width(), m_world.height()), QBrush(QColor(51, 51, 51)));
		QPen origin(Qt::green, 10, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        RendererQtImpl::renderPoint(&painter, &m_world, Vector2(0, 0), origin);

        QPen pen(Qt::green, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        Renderer::render(&painter, &m_world, pen);

        ShapePrimitive primitive;
        primitive.shape = &polygon;
        primitive.rotation = m_angle;
        primitive.transform.set(0, 200);
        RendererQtImpl::renderShape(&painter, &m_world, primitive, pen);
        AABB aabb = AABB::fromShape(primitive);
        pen.setWidth(1);
    	RendererQtImpl::renderAABB(&painter, &m_world, aabb, pen);

    	
    	if(m_lastBody != nullptr)
    	{
            ShapePrimitive shape;
            shape.shape = m_lastBody->shape();
            shape.rotation = m_lastBody->angle();
            shape.transform = m_lastBody->position();
            QPen contact(Qt::red, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
            RendererQtImpl::renderShape(&painter, &m_world, shape, contact);
    	}
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

        //testHit(e->pos());
        
        repaint();
    	
    }

    void Window::keyPressEvent(QKeyEvent *event)
    {
        switch (event->key()) {
        case Qt::Key_R:
        {
            break;
        }
        case Qt::Key_Q:
        {
            break;
        }
        case Qt::Key_D:
        {
            break;
        }
        case Qt::Key_A:
        {
            
            break;
        }
        case Qt::Key_S:
        {
            
            break;
        }
        case Qt::Key_W:
        {
            break;
        }
        default:
            break;
        }
        repaint();
    }

    void Window::keyReleaseEvent(QKeyEvent *event)
    {

        switch (event->key()) {
        case Qt::Key_R:
        {
            m_angle += 1;
            break;
        }
        case Qt::Key_Q:
        {
            m_angle -= 1;
            break;
        }
        case Qt::Key_D:
        {
            break;
        }
        case Qt::Key_A:
        {

            break;
        }
        case Qt::Key_S:
        {

            break;
        }
        case Qt::Key_W:
        {
            break;
        }
        default:
            break;
        }
        repaint();
    }
    void Window::testHit(const QPoint& pos)
    {
        m_lastBody = nullptr;
        Vector2 screen_pos(static_cast<number>(pos.x()), static_cast<number>(pos.y()));
        Vector2 world_pos = m_world.screenToWorld(screen_pos);
        Point point;
        point.setPosition(world_pos);
        ShapePrimitive primitive, shape;
        primitive.shape = &point;
        for (const Body* body : m_world.bodyList())
        {
            shape.transform = body->position();
            shape.rotation = body->angle();
            shape.shape = body->shape();
            auto [isCollide, simplex] = GJK::gjk(shape, primitive);
            if (isCollide)
            {
                m_lastBody = body;
            }
        }
    }
    void Window::testShape(QPainter* painter)
    {
        Rectangle m_rectShape;
        Polygon m_polygon;
        Circle m_circle;
        Ellipse m_ellipse;
        Edge m_edge;
        Curve m_curve;
        ShapePrimitive rect, polygon, circle, ellipse, edge, curve;
        m_rectShape.set(50, 50);
        m_rectShape.scale(2);
        m_polygon.append({ {-4, 1},{-3, -5}, {5, -6}, {7, 4}, {0, 6}, {-4, 1} });
        m_polygon.scale(10);
        m_ellipse.set({ -20, 15 }, { 20, -15 });
        m_ellipse.scale(5);
        m_circle.setRadius(6);
        m_circle.scale(12);
        m_edge.set({ -250, 40 }, { 350, 60 });
        m_curve.set({ -600, 200 }, { -50, 20 }, { 0, 40 }, { 600, 150 });
        

        QPen pen(Qt::green, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);

        rect.shape = &m_rectShape;
        rect.rotation = 45;
        rect.transform.set(180, 120);
        RendererQtImpl::renderRectangle(painter, &m_world, rect, pen);

        pen.setWidth(2);
        polygon.shape = &m_polygon;
        polygon.rotation = 45;
        polygon.transform.set(-240, 180);
        RendererQtImpl::renderPolygon(painter, &m_world, polygon, pen);

        circle.shape = &m_circle;
        circle.transform.set(-200, 250);
        RendererQtImpl::renderCircle(painter, &m_world, circle, pen);

        ellipse.shape = &m_ellipse;
        ellipse.rotation = m_angle;
        ellipse.transform.set(-100, 440);
        RendererQtImpl::renderEllipse(painter, &m_world, ellipse, pen);

        edge.shape = &m_edge;
        edge.transform.set(0, 400);
        RendererQtImpl::renderEdge(painter, &m_world, edge, pen);

        curve.shape = &m_curve;
        curve.transform.set(0, 600);
        RendererQtImpl::renderCurve(painter, &m_world, curve, pen);
    }

    void Window::createStackBox()
    {
    	for(int j = 10;j >= 0;j--)
    	{
    		for(int i = 0;i < 2 * (10 - j) + 1;i++)
    		{
                Body* body = new Body;
                //body->setAngle(j * j + 12);
                body->setShape(&rectangle);
                body->setPosition({static_cast<number>(-55 * (10 - j) + i * 55),static_cast<number>(j * 65 + 65)});
                m_world.addBody(body);
    		}
    	}
    }

}
