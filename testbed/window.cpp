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

		rectangle.set(1, 1);
		land.set(36, 0.2);
		polygon.append({ {3,0}, {2, 3}, {-2, 3}, {-3, 0}, {-2, -3},{2, -3}, {3, 0} });
		polygon.scale(0.2);
		ellipse.set({-4, 3}, {4, -3});
		ellipse.scale(0.1);
		circle.setRadius(0.5);
		//circle.scale(7);
		edge.set({-18, 0}, {18, 0});

		m_world.setEnableGravity(true);
		m_world.setLinearVelocityDamping(0.8f);
		m_world.setAirFrictionCoefficient(0.8f);
		m_world.setAngularVelocityDamping(0.8f);
		testCollision();
		//createStackBox(4, 1.1, 1.1);
		//createBoxesAndGround(12);
		//testPendulum();


		//prim2.bodyA = rect2;
		//prim2.bodyB = rect3;
		//prim2.localPointA.set(0, -0.2);
		//prim2.localPointB.set(0, 0.2);
		//joint2.set(prim2);
		
		prim.bodyA = rect2;
		prim.bodyB = rect3;
		prim.referenceAngle = 0;

		joint.set(prim);

		prim3.minDistance = 4;
		prim3.maxDistance = 6;
		prim3.bodyA = rect2;
		prim3.bodyB = rect3;
		prim3.localPointA.set(0.5, 0.5);
		prim3.localPointB.set(-0.5, -0.5);
		joint3.set(prim3);
		
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

		//for(int i = 0;i < m_world.bodyList().size();i++)
		//{
		//	for(int j = 0;j < m_world.bodyList().size();j++)
		//	{
		//		if(i != j)
		//		{
		//			auto result = Detector::detect(m_world.bodyList()[i], m_world.bodyList()[j]);
		//			if (result.isColliding)
		//				solver.add(result);
		//		}
		//	}
		//}
		auto result1 = Detector::detect(rect2, ground);
		if(result1.isColliding)
			solver.add(result1);
		auto result2 = Detector::detect(rect3, ground);
		if (result2.isColliding)
			solver.add(result2);
		//auto result3 = Detector::detect(rect2, rect3);
		//if (result3.isColliding)
		//	solver.add(result3);

		
		solver.prepare(dt);
		solver.solve(dt);
		
		joint.prepare(dt);
		joint.solveVelocity(dt);
		joint.solvePosition(dt);

		//joint2.prepare(dt);
		//joint2.solveVelocity(dt);
		//joint2.solvePosition(dt);

		joint3.prepare(dt);
		joint3.solveVelocity(dt);
		joint3.solvePosition(dt);
		
		m_world.stepPosition(dt);
		repaint();
	}
	void Window::testCollision()
	{
		rect2 = m_world.createBody();
		rect2->setShape(&rectangle);
		rect2->position().set({ 0, 0 });
		rect2->angle() = 0;
		rect2->setMass(100);
		rect2->setType(Body::BodyType::Static);
		
		rect3 = m_world.createBody();
		rect3->setShape(&rectangle);
		rect3->position().set({ 0, 6 });
		rect3->angle() = 0;
		rect3->setMass(300);
		rect3->setType(Body::BodyType::Dynamic);
		
		rect = m_world.createBody();
		rect->setShape(&rectangle);
		rect->position().set({ 0, 6 });
		rect->angle() = -115;
		rect->setMass(Constant::Max);
		rect->setType(Body::BodyType::Static);
		
		ground = m_world.createBody();
		ground->setShape(&edge);
		ground->position().set({0, -8});
		ground->setMass(Constant::Max);
		ground->setType(Body::BodyType::Static);
		
		//createStackBox(5, 1.1, 1.1);
		
		
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
		rect->setShape(&circle);
		rect->position().set({ 0, 4 });
		rect->angle() = 0;
		rect->setMass(DBL_MAX);
		rect->setType(Body::BodyType::Kinematic);

		rect2 = m_world.createBody();
		rect2->setShape(&rectangle);
		rect2->position().set(-12, -9);
		rect2->angle() = 0;
		rect2->setMass(100);
		rect2->setType(Body::BodyType::Dynamic);
		
	}

	void Window::paintEvent(QPaintEvent*)
	{
		QPainter painter(this);
		//prepare for background, origin and clipping boundary
		painter.setRenderHint(QPainter::Antialiasing);
		painter.setClipRect(m_world.leftTop().x, m_world.leftTop().y, m_world.width(), m_world.height());
		painter.setBackground(QBrush(QColor(50, 50, 50)));
		painter.fillRect(QRectF(m_world.leftTop().x, m_world.leftTop().y, m_world.width(), m_world.height()),
		                 QBrush(QColor(50, 50, 50)));
		QPen origin(Qt::green, 10, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
		RendererQtImpl::renderPoint(&painter, &m_world, Vector2(0, 0), origin);

		QPen pen(Qt::green, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
		Renderer::render(&painter, &m_world, pen);

	}

	void Window::resizeEvent(QResizeEvent* e)
	{
		m_world.setRightBottom(Vector2(e->size().width() - m_world.leftTop().x,
		                               e->size().height() - m_world.leftTop().y));
		this->repaint();
	}

	void Window::mousePressEvent(QMouseEvent*e)
	{

		Vector2 pos(e->pos().x(), e->pos().y());
		mousePos = m_world.screenToWorld(pos);
	}

	void Window::mouseReleaseEvent(QMouseEvent* e)
	{
		Vector2 pos(e->pos().x(), e->pos().y());
		mousePos = m_world.screenToWorld(pos);
		clickPos.clear();
	}
	

	void Window::mouseMoveEvent(QMouseEvent* e)
	{
		//testHit(e->pos());
		
		Vector2 pos(e->pos().x(), e->pos().y());
		mousePos = m_world.screenToWorld(pos);
		//originPoint.set(m_world.screenToWorld(pos));
		repaint();
	}

	void Window::mouseDoubleClickEvent(QMouseEvent* event)
	{
		Vector2 pos(event->x(), event->y());
		clickPos.set(m_world.screenToWorld(pos));
		fmt::print("select body at {}\n",clickPos);
	}

	void Window::keyPressEvent(QKeyEvent* event)
	{
		rect3->velocity() += {0, 9.8};
		//switch (event->key())
		//{
		//case Qt::Key_R:
		//	{
		//		m_angle += 1;
		//		break;
		//	}
		//case Qt::Key_Q:
		//	{
		//		m_angle -= 1;
		//		break;
		//	}
		//case Qt::Key_D:
		//	{
		//	//rect->velocity() += Vector2(5, 0);
		//	rect2->position().set(rect2->position() + Vector2(0.1, 0));
		//		break;
		//	}
		//case Qt::Key_A:
		//	{
		//	//rect->velocity() += Vector2(-5, 0);
		//	rect2->position().set(rect2->position() + Vector2(-0.1, 0));
		//		break;
		//	}
		//case Qt::Key_S:
		//	{
		//		rect2->position().set(rect2->position() + Vector2(0, -0.1));
		//		break;
		//	}
		//case Qt::Key_W:
		//	{
		//		rect2->position().set(rect2->position() + Vector2(0, 0.1));
		//		break;
		//	}
		//case Qt::Key_Space:
		//{
		//	//rect->forces() += Vector2(0, 50);
		//	originPoint.set(originPoint + Vector2(0, 5));
		//	break;
		//}
		//default:
		//	break;
		//}
		repaint();
	}

	void Window::keyReleaseEvent(QKeyEvent* event)
	{
		//switch (event->key())
		//{
		//case Qt::Key_R:
		//{
		//	m_angle += 8;
		//	break;
		//}
		//case Qt::Key_Q:
		//{
		//	m_angle -= 8;
		//	break;
		//}
		//case Qt::Key_D:
		//{
		//	rect->velocity() += Vector2(5, 0);
		//	originPoint.set(originPoint + Vector2(5, 0));
		//	break;
		//}
		//case Qt::Key_A:
		//{
		//	rect->velocity() += Vector2(-5, 0);
		//	originPoint.set(originPoint + Vector2(-5, 0));
		//	break;
		//}
		//case Qt::Key_S:
		//{
		//	originPoint.set(originPoint + Vector2(0, -5));
		//	break;
		//}
		//case Qt::Key_Space:
		//{
		//	rect->forces() += Vector2(0, 50);
		//	originPoint.set(originPoint + Vector2(0, 5));
		//	break;
		//}
		//default:
		//	break;
		//}
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
	
	void Window::createStackBox(const uint16_t& row = 10, const real& margin = 65, const real& spacing = 55)
	{
		for (real j = row; j > 0; j--)
		{
			for (real i = 0; i < 2 * (row - j) + 1; i++)
			{
				Body* body = m_world.createBody();
				body->position().set({
					static_cast<real>(-spacing * (row - j) + i * spacing), static_cast<real>(j * margin + margin) - 6
					});
				body->setShape(&rectangle);
				body->angle() = 0;
				body->setMass(400);
				body->setType(Body::BodyType::Dynamic);
			}
		}


		ground = m_world.createBody();
		ground->setShape(&land);
		ground->position().set({ 0, -8 });
		ground->setMass(100000000);
		ground->setType(Body::BodyType::Static);
	}
	void Window::createBoxesAndGround(const real& count)
	{
		for(real j = 0;j < count;j++)
		{
			Body* body = m_world.createBody();
			body->position().set({1, -6 + j * 1.2});
			body->setShape(&rectangle);
			body->angle() = 0;
			body->setMass(400);
			body->setType(Body::BodyType::Dynamic);
		}


		ground = m_world.createBody();
		ground->setShape(&land);
		ground->position().set({ 0, -8 });
		ground->setMass(100000000);
		ground->setType(Body::BodyType::Static);
	}
}
