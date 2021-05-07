#include "testbed/window.h"

#include "include/render/renderer.h"

namespace Physics2D
{
	Window::Window(QWidget* parent)
	{
		this->setParent(parent);
		this->setWindowTitle("Testbed");
		this->resize(1920, 1080);
		this->setMouseTracking(true);
		m_world.setGeometry({0, 0}, {1920, 1080});

		rectangle.set(2, 2);
		land.set(36, 0.2);
		polygon.append({ {3,0}, {2, 3}, {-2, 3}, {-3, 0}, {-2, -3},{2, -3}, {3, 0} });
		//polygon.scale(0.2);
		ellipse.set({-5, 4}, {5, -4});
		//ellipse.scale(0.1);
		circle.setRadius(0.5);
		//circle.scale(7);
		edge.set({-18, 0}, {18, 0});

		distancePrim.minDistance = 1;
		distancePrim.maxDistance = 1.5;
		distancePrim.localPointA.set(0, 0);
		distancePrim.localPointB.set(0, 0);

		anglePrim.referenceAngle = 45;

		pointPrim.localPointA.set(-0.5, -0.5);
		pointPrim.localPointB.set(0.5, 0.5);

		mousePrim.localPointA.set(0.5, 0.5);
		mousePrim.mousePoint.set(2.0, 2.0);
		
		
		m_world.setEnableGravity(true);
		m_world.setGravity({ 0, -9.8 });
		m_world.setLinearVelocityDamping(0.8f);
		m_world.setAirFrictionCoefficient(0.8f);
		m_world.setAngularVelocityDamping(0.8f);
		//createStackBox(4, 1.1, 1.1);
		//createBoxesAndGround(12);
		//testPendulum();
		//testCollision();
		//testMpr();
		//testJoint();
		testSAT();
		connect(&m_timer, &QTimer::timeout, this, &Window::process);
		m_timer.setInterval(15);
		m_timer.start();

		
	}

	Window::~Window()
	{
	}
	void Window::testMpr()
	{
		ShapePrimitive shape1, shape2;
		shape1.shape = &rectangle;
		shape2.shape = &rectangle;
		shape1.rotation = 45;
		shape1.transform.set(1, 1);
		shape2.rotation = 37;
		shape2.transform.set(1.5, 2);
		auto [centerToOrigin, simplex] = MPR::discover(shape1, shape2);
		auto [isColliding, finalSimplex] = MPR::refine(shape1, shape2, simplex, centerToOrigin);
		fmt::print("collide:{}\n", isColliding);
		fmt::print("A:{}, B:{}, result:{}\n", finalSimplex.vertices[1].pointA, finalSimplex.vertices[1].pointB, finalSimplex.vertices[1].result);
		fmt::print("A:{}, B:{}, result:{}\n", finalSimplex.vertices[2].pointA, finalSimplex.vertices[2].pointB, finalSimplex.vertices[2].result);
	}

	void Window::testSAT()
	{
		ShapePrimitive shape1, shape2;
		shape1.shape = &rectangle;
		shape2.shape = &rectangle;
		shape1.rotation = 45;
		shape1.transform.set(1, 1);
		shape2.rotation = 37;
		shape2.transform.set(3, 1);
		SATResult result = SAT::polygonVsPolygon(shape2, shape1);
		fmt::print("collide:{}, normal:{}, penetration:{}\n", result.isColliding, result.normal, result.penetration);
		fmt::print("point pair: {}, {}\n", result.pointPair.pointA, result.pointPair.pointB);
	}

	void Window::process()
	{
		const real dt = 1.0 / 60.0;
		const real inv_dt = 60;
		m_world.stepVelocity(dt);

		for (Joint* joint : m_world.jointList())
			joint->prepare(dt);
		
		for (Joint* joint : m_world.jointList())
			fmt::print("{}\n", joint->solveVelocity(dt));
		
		
		m_world.stepPosition(dt);
		repaint();
	}
	void Window::testJoint()
	{
		rect = m_world.createBody();
		rect->setShape(&rectangle);
		rect->position().set({ 0, 0 });
		rect->angle() = 0;
		rect->setMass(5);
		rect->setType(Body::BodyType::Dynamic);
		
		rect2 = m_world.createBody();
		rect2->setShape(&rectangle);
		rect2->position().set({ 0, -5 });
		rect2->angle() = 0;
		rect2->setMass(100);
		rect2->setType(Body::BodyType::Dynamic);
		
		rect3 = m_world.createBody();
		rect3->setShape(&circle);
		rect3->position().set({ 0, -8 });
		rect3->angle() = 0;
		rect3->setMass(100);
		rect3->setType(Body::BodyType::Dynamic);

		mousePrim.bodyA = rect;
		MouseJoint* j = m_world.createJoint(mousePrim);
		
	}
	void Window::testCollision()
	{
		rect2 = m_world.createBody();
		rect2->setShape(&rectangle);
		rect2->position().set({ 0, 2 });
		rect2->angle() = 5;
		rect2->setMass(300);
		rect2->setType(Body::BodyType::Dynamic);
		
		rect3 = m_world.createBody();
		rect3->setShape(&circle);
		rect3->position().set({ 3, 0 });
		rect3->angle() = 0;
		rect3->setMass(300);
		rect3->setType(Body::BodyType::Static);
		
		rect = m_world.createBody();
		rect->setShape(&circle);
		rect->position().set({ 0, 6 });
		rect->angle() = -115;
		rect->setMass(Constant::Max);
		rect->setType(Body::BodyType::Static);
		
		ground = m_world.createBody();
		ground->setShape(&edge);
		ground->position().set({0, -4});
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

		QColor color = Qt::green;
		color.setAlphaF(0.6);
		pen.setColor(color);
		for(int i = -10;i <= 10;i++)
		{
			RendererQtImpl::renderPoint(&painter, &m_world, Vector2(0, i), pen);
			RendererQtImpl::renderPoint(&painter, &m_world, Vector2(i, 0), pen);
		}
		color.setAlphaF(0.45);
		pen.setColor(color);
		pen.setWidth(1);
		RendererQtImpl::renderLine(&painter, &m_world, Vector2(0, -10), Vector2(0, 10), pen);
		RendererQtImpl::renderLine(&painter, &m_world, Vector2(-10, 0), Vector2(10, 0), pen);
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
		Body* nb = m_world.createBody();
		nb->setShape(&circle);
		nb->position().set({ 0, -2 });
		nb->angle() = 5;
		nb->setMass(300);
		nb->setType(Body::BodyType::Dynamic);
		
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
		//mousePrim.mousePoint = mousePos;
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
		//rect3->velocity() += {0, 9.8};
		//mousePrim.mousePoint -= {-1, 1};
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

	void Window::wheelEvent(QWheelEvent* event)
	{

		
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
