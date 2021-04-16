#include "testbed/window.h"

namespace Physics2D
{
	Window::Window(QWidget* parent)
	{
		this->setParent(parent);
		this->setWindowTitle("Testbed");
		this->resize(1920, 1080);
		this->setMouseTracking(true);
		m_world.setGeometry({0, 0}, {1920, 1080});

		rectangle.set(50, 25);
		//rectangle.scale(2);
		land.set(1600, 10);

		polygon.append({ {-25,0}, {25, -12.5}, {25, 12.5}, {-25, 0} });
		polygon.scale(2);
		ellipse.set({-40, 30}, {40, -30});
		circle.setRadius(6);
		circle.scale(12);
		edge.set({-800, 30}, {800, 30});
		curve.set({-600, 200}, {-50, 20}, {0, 40}, {600, 150});

		m_world.setEnableGravity(true);
		m_world.setLinearVelocityDamping(0.9f);
		m_world.setAirFrictionCoefficient(0.1f);
		m_world.setAngularVelocityDamping(0.01f);
		//createStackBox(5, 25, 25);
		targetPoint.set({25, 0});
		testPendulum();
		connect(&m_timer, &QTimer::timeout, this, &Window::process);
		m_timer.setInterval(15);
		m_timer.start();
	}

	Window::~Window()
	{
	}

	void Window::process()
	{
		const real dt = 1.0f / 60.0f;
		const real inv_dt = 60.0f;
		m_world.stepVelocity(dt);
		m_rectCenter.emplace_back(Matrix2x2(rect->angle()).multiply(targetPoint) + rect->position());
		DistanceConstraintPrimitive p, p2;
		p.source = rect;
		p.sourcePoint = targetPoint;
		p.targetPoint = originPoint;
		p.distance = 100;

		
		
		DistanceConstraintSolver solver;
		solver.solve(p, dt);
		
		m_world.stepPosition(dt);
		
		repaint();
	}
	void Window::testCollision()
	{
		ground = m_world.createBody();
		ground->setShape(&land);
		ground->position().set({0, 100});
		ground->setMass(50);

		rect = m_world.createBody();
		rect->setShape(&rectangle);
		rect->position().set({0, 800});
		rect->angle() = 45;
		rect->setMass(2);
		rect->setType(Body::BodyType::Dynamic);
	}

	void Window::createSnakeBody()
	{
		for (size_t i = 0; i < 15; i++)
		{
			Body* rect = m_world.createBody();
			rect->setShape(&rectangle);
			rect->position().set({-600 + static_cast<real>(i * 60), 250});
			rect->angle() = 45;
			rect->setMass(2);
			rect->setType(Body::BodyType::Dynamic);
			if (i == 0)
				rect2 = rect;
		}
	}
	void Window::testDistanceJoint()
	{
	}
	void Window::testPendulum()
	{
		rect = m_world.createBody();
		rect->setShape(&rectangle);
		rect->position().set({ -800, 800 });
		rect->angle() = 0;
		rect->setMass(5);
		rect->setType(Body::BodyType::Dynamic);
		
	}

	void Window::paintEvent(QPaintEvent*)
	{
		QPainter painter(this);
		//prepare for background, origin and clipping boundary
		painter.setRenderHint(QPainter::Antialiasing);
		painter.setClipRect(m_world.leftTop().x, m_world.leftTop().y, m_world.width(), m_world.height());
		painter.fillRect(QRectF(m_world.leftTop().x, m_world.leftTop().y, m_world.width(), m_world.height()),
		                 QBrush(QColor(50, 50, 50)));
		QPen origin(Qt::green, 10, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
		RendererQtImpl::renderPoint(&painter, &m_world, Vector2(0, 0), origin);

		QPen pen(Qt::green, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
		Renderer::render(&painter, &m_world, pen);


		RendererQtImpl::renderPoint(&painter, &m_world, originPoint, pen);
		Vector2 pos = Matrix2x2(rect->angle()).multiply(targetPoint) + rect->position();
		if(rect != nullptr)
			RendererQtImpl::renderLine(&painter, &m_world, pos, originPoint, pen);

		pen.setWidth(1);
		for(const Vector2& p: m_rectCenter)
		{
			RendererQtImpl::renderPoint(&painter, &m_world, p, pen);
		}
	}

	void Window::resizeEvent(QResizeEvent* e)
	{
		m_world.setRightBottom(Vector2(e->size().width() - m_world.leftTop().x,
		                               e->size().height() - m_world.leftTop().y));
		this->repaint();
	}

	void Window::mousePressEvent(QMouseEvent*)
	{
	}

	void Window::mouseReleaseEvent(QMouseEvent* e)
	{
	}

	void Window::mouseMoveEvent(QMouseEvent* e)
	{
		//testHit(e->pos());
		
		//Vector2 pos(e->pos().x(), e->pos().y());
		//originPoint.set(m_world.screenToWorld(pos));
		repaint();
	}

	void Window::keyPressEvent(QKeyEvent* event)
	{
		switch (event->key())
		{
		case Qt::Key_R:
			{
				m_angle += 8;
				break;
			}
		case Qt::Key_Q:
			{
				m_angle -= 8;
				break;
			}
		case Qt::Key_D:
			{
			rect->velocity() += Vector2(5, 0);
				originPoint.set(originPoint + Vector2(5, 0));
				break;
			}
		case Qt::Key_A:
			{
			rect->velocity() += Vector2(-5, 0);
				originPoint.set(originPoint + Vector2(-5, 0));
				break;
			}
		case Qt::Key_S:
			{
				originPoint.set(originPoint + Vector2(0, -5));
				break;
			}
		case Qt::Key_W:
			{
				originPoint.set(originPoint + Vector2(0, 5));
				break;
			}
		case Qt::Key_Space:
		{
			rect->forces() += Vector2(0, 50);
			originPoint.set(originPoint + Vector2(0, 5));
			break;
		}
		default:
			break;
		}
		repaint();
	}

	void Window::keyReleaseEvent(QKeyEvent* event)
	{
		switch (event->key())
		{
		case Qt::Key_R:
		{
			m_angle += 8;
			break;
		}
		case Qt::Key_Q:
		{
			m_angle -= 8;
			break;
		}
		case Qt::Key_D:
		{
			rect->velocity() += Vector2(5, 0);
			originPoint.set(originPoint + Vector2(5, 0));
			break;
		}
		case Qt::Key_A:
		{
			rect->velocity() += Vector2(-5, 0);
			originPoint.set(originPoint + Vector2(-5, 0));
			break;
		}
		case Qt::Key_S:
		{
			originPoint.set(originPoint + Vector2(0, -5));
			break;
		}
		case Qt::Key_Space:
		{
			rect->forces() += Vector2(0, 50);
			originPoint.set(originPoint + Vector2(0, 5));
			break;
		}
		default:
			break;
		}
		repaint();
	}

	void Window::testBVH(QPainter* painter)
	{
	}

	void Window::testHit(const QPoint& pos)
	{
		m_lastBody = nullptr;
		Vector2 screen_pos(static_cast<real>(pos.x()), static_cast<real>(pos.y()));
		Vector2 world_pos = m_world.screenToWorld(screen_pos);
		Point point;
		point.setPosition(world_pos);
		ShapePrimitive primitive, shape;
		primitive.shape = &point;
		for (Body* body : m_world.bodyList())
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

		if (m_lastBody != nullptr)
		{
			ShapePrimitive shape;
			shape.shape = m_lastBody->shape();
			shape.rotation = m_lastBody->angle();
			shape.transform = m_lastBody->position();
			QPen contact(Qt::red, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
			//RendererQtImpl::renderShape(&painter, &m_world, shape, contact);
		}
	}

	void Window::testAABB(QPainter* painter)
	{
		QPen pen(Qt::green, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
		ShapePrimitive primitive;
		primitive.shape = &rectangle;
		primitive.rotation = m_angle;
		primitive.transform.set(0, 200);
		RendererQtImpl::renderShape(painter, &m_world, primitive, pen);
		AABB aabb = AABB::fromShape(primitive, 1.05);
		pen.setWidth(1);
		RendererQtImpl::renderAABB(painter, &m_world, aabb, pen);

		ShapePrimitive primitive2;
		primitive2.shape = &polygon;
		primitive2.rotation = m_angle;
		primitive2.transform.set(150, 150);
		pen.setWidth(2);
		RendererQtImpl::renderShape(painter, &m_world, primitive2, pen);
		AABB aabb_ell = AABB::fromShape(primitive2, 1.05);
		pen.setWidth(1);
		RendererQtImpl::renderAABB(painter, &m_world, aabb_ell, pen);

		AABB uni = aabb.unite(aabb_ell);
		uni.scale(1.05);
		pen.setWidth(1);
		if (aabb.collide(aabb_ell))
			pen.setColor(Qt::red);
		RendererQtImpl::renderAABB(painter, &m_world, uni, pen);
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
		m_polygon.append({{-4, 1}, {-3, -5}, {5, -6}, {7, 4}, {0, 6}, {-4, 1}});
		m_polygon.scale(10);
		m_ellipse.set({-20, 15}, {20, -15});
		m_ellipse.scale(5);
		m_circle.setRadius(6);
		m_circle.scale(12);
		m_edge.set({-250, 40}, {350, 60});
		m_curve.set({-600, 200}, {-50, 20}, {0, 40}, {600, 150});


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

	void Window::createStackBox(const uint16_t& row = 10, const uint16_t& margin = 65, const uint16_t& spacing = 55)
	{
		for (int j = row; j > 0; j--)
		{
			for (int i = 0; i < 2 * (row - j) + 1; i++)
			{
				Body* body = m_world.createBody();
				//body->setAngle(j * j + 12);
				body->setShape(&rectangle);
				body->position().set({
					static_cast<real>(-spacing * (row - j) + i * spacing), static_cast<real>(j * margin + margin)
				});
			}
		}
	}
}
