#include "testbed/window.h"


#include "include/dynamics/constraint/constraint.h"
#include "include/dynamics/constraint/contact.h"
#include "include/render/renderer.h"
#include <iostream>
namespace Physics2D
{


	Window::Window(QWidget* parent)
	{
		this->setParent(parent);
		this->setWindowTitle("Testbed");
		this->resize(1920, 1080);
		this->setMouseTracking(true);

		brick.set(1.5f, 0.5f);
		rectangle.set(1.0f, 1.0f);
		land.set(32.0f, 0.2f);
		polygon.append({{3, 0}, {2, 3}, {-2, 3}, {-3, 0}, {-2, -3}, {2, -3}, {3, 0}});
		//polygon.append({ {-2, 0}, {0, 4}, {4, 6}, {10, 4}, {4, -2}, {-2, 0} });
		//polygon.append({ {0, 6}, {6, -4}, {-6, -4}, {0, 6}});
		polygon.scale(0.16f);
		ellipse.set({-5, 4}, {5, -4});
		ellipse.scale(0.1f);
		circle.setRadius(0.5f);
		//circle.scale(7);
		edge.set({-32, 0}, {32, 0});
		capsule.set(1, 2);
		sector.set(Math::degreeToRadian(0), Math::degreeToRadian(90), 2);

		boxHorizontal.set({ -roomSize, 0 }, { roomSize, 0 });
		boxVertical.set({ 0, roomSize }, { 0, -roomSize });

		rectangle_ptr = std::make_shared<Rectangle>(rectangle);
		land_ptr = std::make_shared<Rectangle>(land);
		polygon_ptr = std::make_shared<Polygon>(polygon);
		ellipse_ptr = std::make_shared<Ellipse>(ellipse);
		circle_ptr = std::make_shared<Circle>(circle);
		edge_ptr = std::make_shared<Edge>(edge);
		capsule_ptr = std::make_shared<Capsule>(capsule);
		sector_ptr = std::make_shared<Sector>(sector);
		brick_ptr = std::make_shared<Rectangle>(brick);
		

		horizontalWall = std::make_shared<Edge>(boxHorizontal);
		verticalWall = std::make_shared<Edge>(boxVertical);


		m_world.setEnableGravity(true);
		m_world.setGravity({0, -4.0f});
		m_world.setLinearVelocityDamping(0.1f);
		m_world.setAirFrictionCoefficient(0.8f);
		m_world.setAngularVelocityDamping(0.1f);
		m_world.setPositionIteration(8);
		m_world.setVelocityIteration(6);

		pointPrim.bodyA = nullptr;
		mj = m_world.createJoint(pointPrim);
		mj->setActive(false);
		//createStackBox(6, 1.1, 1.1);
		//createBoxRoom();
		//createPyramid();
		createGround();
		//createBoxesAndGround(32);
		//testPendulum();
		//testCollision();
		//testJoint();
		createBridge();


		//testBroadphase();
		//testCCD();
		//testCapsule();
		//testRaycast();
		//testTree();
		
		camera.setViewport(Utils::Camera::Viewport((0, 0), (1920, 1080)));
		camera.setWorld(&m_world);
		camera.setDbvh(&dbvh);
		camera.setTree(&tree);
		
		camera.setAabbVisible(false);
		camera.setDbvhVisible(false);
		camera.setTreeVisible(false);
		camera.setAxisVisible(false);
		camera.setGridScaleLineVisible(false);
		connect(&m_timer, &QTimer::timeout, this, &Window::process);
		connect(&m_paintTimer, &QTimer::timeout, this, [=]
		{
				this->repaint();
		});
		//isStop = true;
		
		m_timer.setInterval(15);
		m_timer.start();
		m_paintTimer.setInterval(15);
		m_paintTimer.start();
		
	}

	Window::~Window()
	{

	}
	void Window::createGround()
	{
		ground = m_world.createBody();
		ground->setShape(edge_ptr);
		ground->position().set({ 0, -15.0 });
		ground->setMass(Constant::Max);
		ground->setType(Body::BodyType::Static);
		tree.insert(ground);
	}
	void Window::testTree()
	{
		ground = m_world.createBody();
		ground->setShape(edge_ptr);
		ground->position().set({ 0, 0 });
		ground->setMass(Constant::Max);
		ground->setType(Body::BodyType::Static);
		ground->setFriction(0.7f);

		tree.insert(ground);

		for(real i = 0;i < 6;i++)
		{
			rect = m_world.createBody();
			rect->setShape(capsule_ptr);
			rect->position().set({ -5 + 2 * i, 5 + i * 2 });
			rect->rotation() = 0;
			rect->setMass(200);
			rect->setType(Body::BodyType::Dynamic);
			rect->setFriction(0.1f);
			tree.insert(rect);
		}


	}
	void Window::createPyramid()
	{
		real offset = 0.0f;
		real max = 20.0f;
		for(real j = 0;j < max;j += 1.0f)
		{
			for (real i = 0.0; i < max - j; i += 1.0f)
			{
				Body* body = m_world.createBody();
				body->position().set({ -12.0f + i * 1.1f + offset, j * 1.1f + 3.0f });
				body->setShape(rectangle_ptr);
				body->rotation() = 0;
				body->setMass(1.0f);
				body->setType(Body::BodyType::Dynamic);
				body->setFriction(0.8f);
				body->setRestitution(0.0f);
				camera.setTargetBody(body);
				tree.insert(body);
			}
			offset += 0.5f;
		}
	}
	void Window::createBoxRoom()
	{


		ground = m_world.createBody();
		ground->setShape(horizontalWall);
		ground->position().set({ 0, (roomSize + 0.04f) * 2.0f });
		ground->setMass(Constant::Max);
		ground->setType(Body::BodyType::Static);
		//camera.setMeterToPixel(120);
		tree.insert(ground);

		ground = m_world.createBody();
		ground->setShape(verticalWall);
		ground->position().set({ (roomSize + 0.04f), (roomSize + 0.04f) });
		ground->setMass(Constant::Max);
		ground->setType(Body::BodyType::Static);
		//camera.setMeterToPixel(120);
		tree.insert(ground);

		ground = m_world.createBody();
		ground->setShape(verticalWall);
		ground->position().set({ -(roomSize + 0.04f), (roomSize + 0.04f) });
		ground->setMass(Constant::Max);
		ground->setType(Body::BodyType::Static);
		camera.setTargetBody(ground);
		//camera.setMeterToPixel(120);
		tree.insert(ground);

		ground = m_world.createBody();
		ground->setShape(horizontalWall);
		ground->position().set({ 0, -(0 + 0.04f) });
		ground->setMass(Constant::Max);
		ground->setType(Body::BodyType::Static);
		//camera.setMeterToPixel(120);
		tree.insert(ground);
	}
	void Window::testBroadphase()
	{
		for (int i = 0; i < 500; i++)
		{
			Body* body = m_world.createBody();
			body->position().set(-9.0f + QRandomGenerator::global()->bounded(18.0f),
			                     -9.0f + QRandomGenerator::global()->bounded(18.0f));
			body->setShape(rectangle_ptr);
			body->rotation() = -360 + QRandomGenerator::global()->bounded(720);
			body->setMass(400);
			body->setType(Body::BodyType::Static);
			
			dbvh.insert(body);
		}
	}

	void Window::testCCD()
	{
		rect = m_world.createBody();
		rect->setShape(rectangle_ptr);
		rect->position().set({ 8, -5.4f });
		rect->setMass(Constant::Max);
		rect->rotation() = 90;
		rect->setType(Body::BodyType::Dynamic);

		rect2 = m_world.createBody();
		rect2->setShape(rectangle_ptr);
		rect2->position().set({ -5.5f, -6 });
		rect2->rotation() = 90;
		rect2->setMass(200);
		rect2->setType(Body::BodyType::Bullet);
		rect2->velocity() = { 1000, 0 };
		rect2->angularVelocity() = 0;

		rect3 = m_world.createBody();
		rect3->setShape(rectangle_ptr);
		rect3->position().set({ 8, -6.6f });
		rect3->rotation() = 180;
		rect3->setMass(200);
		rect3->setType(Body::BodyType::Dynamic);
		//rect3->angularVelocity() = 360;
		
		dbvh.insert(rect);
		dbvh.insert(rect2);
		dbvh.insert(rect3);
	}

	void Window::testRaycast()
	{
		isStop = true;
		for (real j = 0; j < 20.0f; j++)
		{
			for (real i = 0; i < 20.0f; i++)
			{
				Body* body = m_world.createBody();
				body->position().set({ i * 0.5f - 8.0f, j * 0.5f + 5.0f });
				body->setShape(rectangle_ptr);
				body->rotation() = 0.0f;
				body->setMass(200.0f);
				body->setType(Body::BodyType::Static);
				body->setFriction(0.8f);
				camera.setTargetBody(body);
				tree.insert(body);
			}
		}
	}

	void Window::testCapsule()
	{
		rect = m_world.createBody();
		rect->setShape(capsule_ptr);
		rect->position().set({ 0, 0 });
		rect->rotation() = 0;
		rect->setMass(20);
		rect->setType(Body::BodyType::Dynamic);
		
		dbvh.insert(rect);
	}

	void Window::createBridge()
	{
		real half = brick_ptr->width() / 2.0f;
		rect = m_world.createBody();
		rect->setShape(brick_ptr);
		rect->position().set({ -5.0f, 0.0f });
		rect->rotation() = 0;
		rect->setMass(1.0f);
		rect->setRestitution(0.2f);
		rect->setFriction(0.8f);
		rect->setType(Body::BodyType::Dynamic);


		PointJointPrimitive ppm;
		ppm.bodyA = rect;
		ppm.localPointA.set(-half, 0);
		ppm.targetPoint.set(-5.0f - half, 0.0f);
		ppm.dampingRatio = 0.2f;
		ppm.frequency = 2;
		m_world.createJoint(ppm);
		real max = 14.0f;
		tree.insert(rect);
		for(real i = 1.0f;i < max;i += 1.0f)
		{
			rect2 = m_world.createBody();
			rect2->setShape(brick_ptr);
			rect2->position().set({ -5.0f + i * brick_ptr->width(), 0.0f });
			rect2->rotation() = 0;
			rect2->setMass(1.0f);
			rect2->setFriction(0.1f);
			rect2->setType(Body::BodyType::Dynamic);

			tree.insert(rect2);
			RevoluteJointPrimitive revolutePrim;
			revolutePrim.bodyA = rect;
			revolutePrim.bodyB = rect2;
			revolutePrim.localPointA.set(half, 0);
			revolutePrim.localPointB.set(-half, 0);
			revolutePrim.dampingRatio = 0.8f;
			revolutePrim.frequency = 2;
			rj = m_world.createJoint(revolutePrim);
			rect = rect2;
		}
		
		//ppm.bodyA = rect2;
		//ppm.localPointA.set(0.75f, 0);
		//ppm.targetPoint.set(-5.0f + max * brick_ptr->width(), 0.0f);
		//ppm.dampingRatio = 0.2f;
		//ppm.frequency = 2;
		//m_world.createJoint(ppm);
	}


	void Window::process()
	{
		if (isStop)
		{
			for (auto& body : m_world.bodyList())
				tree.update(body.get());

			return;
		}
		const real dt = 1.0f / 60.0f;

		m_world.stepVelocity(dt);


		auto potentialList = tree.generate();
		for (auto pair : potentialList)
		{
			auto result = Detector::detect(pair.first, pair.second);
			if (result.isColliding)
				contactMaintainer.add(result);
		}
		contactMaintainer.clearInactivePoints();
		m_world.prepareVelocityConstraint(dt);
		for(int i = 0;i < m_world.velocityIteration(); ++i)
		{
			m_world.solveVelocityConstraint(dt);
			contactMaintainer.solveVelocity(dt);
		}

		m_world.stepPosition(dt);

		for(int i = 0;i < m_world.positionIteration(); ++i)
		{
			contactMaintainer.solvePosition(dt);
			m_world.solvePositionConstraint(dt);
		}
		contactMaintainer.deactivateAllPoints();

		for (auto& body : m_world.bodyList())
			tree.update(body.get());

		
		
	}

	void Window::testJoint()
	{

		ground = m_world.createBody();
		ground->setShape(edge_ptr);
		ground->position().set({ 0, -10 });
		ground->setMass(Constant::Max);
		ground->setType(Body::BodyType::Static);
		ground->setRestitution(1.0f);
		
		rect = m_world.createBody();
		rect->setShape(brick_ptr);
		rect->position().set({-0.5, -1});
		rect->rotation() = 0;
		rect->setMass(1.0f);
		rect->setRestitution(0.2f);
		rect->setFriction(0.8f);
		rect->setType(Body::BodyType::Dynamic);

		rect2 = m_world.createBody();
		rect2->setShape(brick_ptr);
		rect2->position().set({-5, -5});
		rect2->rotation() = 0;
		rect2->setMass(1.0f);
		rect2->setFriction(0.4f);
		rect2->setType(Body::BodyType::Dynamic);



		RevoluteJointPrimitive revolutePrim;
		revolutePrim.bodyA = rect;
		revolutePrim.bodyB = rect2;
		revolutePrim.localPointA.set(-0.75f, 0);
		revolutePrim.localPointB.set(0.75f, 0);
		revolutePrim.dampingRatio = 0.1f;
		revolutePrim.frequency = 2;
		rj = m_world.createJoint(revolutePrim);

		//rotationPrim.bodyA = rect;
		//rotationPrim.bodyB = rect2;
		//rotationPrim.referenceRotation = Math::degreeToRadian(45);
		//RotationJoint* rj = m_world.createJoint(rotationPrim);

		//distancePrim.bodyA = rect;
		//distancePrim.localPointA.set({ 0, 0 });
		//distancePrim.minDistance = 3;
		//distancePrim.maxDistance = 7;
		//distancePrim.targetPoint.set({ 1, 1 });
		//
		//m_world.createJoint(distancePrim);

		//orientationPrim.bodyA = rect;
		//orientationPrim.targetPoint.set({ 1, 1 });
		//orientationPrim.referenceRotation = Math::degreeToRadian(90);
		//m_world.createJoint(orientationPrim);

		//pointPrim.localPointA.set(0, 0);
		//pointPrim.targetPoint.set(0, 0);
		//pointPrim.bodyA = rect;
		//mj = m_world.createJoint(pointPrim);

		

		tree.insert(rect);
		tree.insert(ground);
		tree.insert(rect2);
		camera.setTargetBody(rect);

		//rect->velocity() += {10, 0};

	}

	void Window::testCollision()
	{
		ground = m_world.createBody();
		ground->setShape(edge_ptr);
		ground->position().set({ 0, 0 });
		ground->setMass(Constant::Max);
		ground->setType(Body::BodyType::Static);
		ground->setFriction(0.7f);
		ground->setRestitution(1.0);
		tree.insert(ground);

		rect = m_world.createBody();
		rect->setShape(rectangle_ptr);
		rect->position().set({ -5, 6 });
		rect->rotation() = 2.21805891827f;
		rect->setMass(1);
		rect->setType(Body::BodyType::Dynamic);
		rect->setFriction(0.4f);
		rect->setRestitution(0.0f);
		tree.insert(rect);


		//rect2 = m_world.createBody();
		//rect2->setShape(circle_ptr);
		//rect2->position().set({ 0, 2 });
		//rect2->rotation() = -20;
		//rect2->setMass(200);
		//rect2->setType(Body::BodyType::Static);
		//dbvh.insert(rect2);
		//rect->velocity().set(0.5, 0);
		//rect2->velocity().set(-0.5, 0);

		//rect->angularVelocity() = 15;
		//rect2->angularVelocity() = -15;

		//camera.setTargetBody(rect);

		//rect->velocity().set(0, -4);
	}

	void Window::createSnakeBody()
	{
		for (size_t i = 0; i < 15; i++)
		{
			Body* rect = m_world.createBody();
			rect->setShape(rectangle_ptr);
			rect->position().set({-600 + static_cast<real>(i * 60), 250});
			rect->rotation() = 45;
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
		rect->setShape(circle_ptr);
		rect->position().set({0, 4});
		rect->rotation() = 0;
		rect->setMass(Constant::Max);
		rect->setType(Body::BodyType::Kinematic);

		rect2 = m_world.createBody();
		rect2->setShape(rectangle_ptr);
		rect2->position().set(-12, -9);
		rect2->rotation() = 0;
		rect2->setMass(100);
		rect2->setType(Body::BodyType::Dynamic);
	}

	void Window::paintEvent(QPaintEvent*)
	{
		QPainter painter(this);
		camera.render(&painter);
		
		//std::vector<Vector2> polygon1 = { {0, 6}, {-4, 4},{-6, 2}, {-6, 0}, {-4, -2},{4, -4},{6, 2},{4, 6}, {0, 6} };
		//std::vector<Vector2> polygon2 = { {-10, 1}, {-8, 6},{0, 4},{6, -2}, {-8, -1}, {-10, 1} };
		//std::vector<std::pair<Vector2, Vector2>> lines1, lines2, lines3;
		//for (size_t i = 0; i < polygon1.size() - 1; i++)
		//{
		//	lines1.emplace_back(std::make_pair(polygon1[i], polygon1[i + 1]));
		//}
		//for (size_t i = 0; i < polygon2.size() - 1; i++)
		//{
		//	lines2.emplace_back(std::make_pair(polygon2[i], polygon2[i + 1]));
		//}
		//QColor colorAccurate(Qt::green);
		//QColor colorApproximate1(Qt::red);
		//QColor colorApproximate2("#03A9F4");
		//QColor colorApproximate3(Qt::magenta);
		//QColor colorApproximate4(Qt::yellow);
		//colorAccurate.setAlphaF(0.8);
		//colorApproximate1.setAlphaF(1);
		//colorApproximate2.setAlphaF(1);
		//colorApproximate3.setAlphaF(1);
		//colorApproximate4.setAlphaF(1);
		//QPen penAccurate(colorAccurate, 4, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
		//QPen penApproximate1(colorApproximate1, 4, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
		//QPen penApproximate2(colorApproximate2, 4, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
		//QPen penApproximate3(colorApproximate3, 8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
		//QPen penApproximate4(colorApproximate4, 8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
		//RendererQtImpl::renderLines(&painter, &camera, lines1, penApproximate2);
		//RendererQtImpl::renderLines(&painter, &camera, lines2, penApproximate3);
		//std::vector<Vector2> results = Clipper::sutherlandHodgmentPolygonClipping(polygon1, polygon2);
		//for (size_t i = 0; i < results.size() - 1; i++)
		//{
		//	lines3.emplace_back(std::make_pair(results[i], results[i + 1]));
		//}
		//RendererQtImpl::renderLines(&painter, &camera, lines3, penAccurate);

		//real y0 = 1;
		//real y = 0;
		//std::vector<std::pair<Vector2, Vector2>> lines;
		//lines.reserve(100);

		//real x = 0;
		//real h = 0.1;
		//real yp, yc;
		//real k1, k2, k3, k4;
		////for(int i = 0;i < 50;i++)
		////{
		////	y = y0 + h * (-y0 * y0);
		////	Vector2 p1(x, y0);
		////	x += h;
		////	y0 = y;

		////	y = y0 + h * (-y0 * y0);
		////	Vector2 p2(x, y0);
		////	x += h;
		////	y0 = y;

		////	lines.emplace_back(std::make_pair(p1, p2));
		////}
		////RendererQtImpl::renderLines(&painter, &camera, lines, penApproximate1);
		//lines.clear();
		//x = 0;
		//y0 = 0;
		//y = 0;
		//for (int i = 0; i < 100; i++)
		//{
		//	y = std::sin(x);
		//	Vector2 p1(x, y);
		//	x += h;
		//	y = std::sin(x);
		//	Vector2 p2(x, y);

		//	lines.emplace_back(std::make_pair(p1, p2));
		//}
		//RendererQtImpl::renderLines(&painter, &camera, lines, penAccurate);
		//lines.clear();
		//x = 0;
		//y0 = 0;
		//y = 0;
		//for(int i = 0;i < 100;i++)
		//{
		//	Vector2 p1(x, y);
		//	y = y0 + h * std::cos(x);
		//	x += h;
		//	y0 = y;
		//	Vector2 p2(x, y);

		//	lines.emplace_back(std::make_pair(p1, p2));
		//}
		//RendererQtImpl::renderLines(&painter, &camera, lines, penApproximate1);
		//lines.clear();
		//x = 0;
		//y0 = 0;
		//y = 0;
		//for (int i = 0; i < 100; i++)
		//{
		//	Vector2 p1(x, y);
		//	yp = y0 + h * std::cos(x);
		//	yc = y0 + h * std::cos(yp);
		//	y = 0.5 * (yp + yc);
		//	x += h;
		//	y0 = y;
		//	Vector2 p2(x, y);
		//	lines.emplace_back(std::make_pair(p1, p2));

		//}
		//RendererQtImpl::renderLines(&painter, &camera, lines, penApproximate2);
		//lines.clear();
		//x = 0;
		//y0 = 0;
		//y = 0;
		//for(int i = 0;i < 100;i++)
		//{
		//	Vector2 p1(x, y);
		//	k1 = std::cos(x);
		//	k2 = std::cos(x + k1 * h / 2.0);
		//	k3 = std::cos(x + k2 * h / 2.0);
		//	k4 = std::cos(x + k3 * h);
		//	y = y0 + h * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0;
		//	x += h;
		//	Vector2 p2(x, y);
		//	y0 = y;
		//	lines.emplace_back(std::make_pair(p1, p2));
		//}
		//RendererQtImpl::renderLines(&painter, &camera, lines, penApproximate3);
		//lines.clear();
		//x = 0;
		//y0 = 0;
		//y = 0;
		//
		//for(int i = 1;i < 4;i++)
		//{
		//	Vector2 p1(x, y);
		//	k1 = std::cos(x);
		//	k2 = std::cos(x + k1 * h / 2.0);
		//	k3 = std::cos(x + k2 * h / 2.0);
		//	k4 = std::cos(x + k3 * h);
		//	y = y0 + h * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0;
		//	x += h;
		//	Vector2 p2(x, y);
		//	y0 = y;
		//	lines.emplace_back(std::make_pair(p1, p2));
		//}
		//for (int i = 0; i < 96; i++)
		//{
		//	Vector2 p1(x, y);
		//	y = y0 + (h / 24.0) * (55 * std::cos(x) - 59 * std::cos(x - h * 1) + 37 * std::cos(x - h * 2) - 9 * std::cos(x - h * 3));
		//	x += h;
		//	Vector2 p2(x, y);
		//	y0 = y;
		//	lines.emplace_back(std::make_pair(p1, p2));
		//}
		//RendererQtImpl::renderLines(&painter, &camera, lines, penApproximate4);
		//lines.clear();
		//x = 0;
		//y0 = 1;
		//y = 0;
		//for (int i = 0; i < 50; i++)
		//{
		//	y = std::sqrt(10.0 * y0 + 25.0) - 5.0;
		//	Vector2 p1(x, y);
		//	x += h;
		//	y0 = y;

		//	y = std::sqrt(10.0 * y0 + 25.0) - 5.0;
		//	Vector2 p2(x, y);
		//	x += h;
		//	y0 = y;
		//	lines.emplace_back(std::make_pair(p1, p2));
		//}
		//RendererQtImpl::renderLines(&painter, &camera, lines, penApproximate2);

		//for(int i = 0; i < 100;i++)
		//{

		//	
		//	//points.emplace_back(Vector2(x, std::exp(x)));
		//	x += h;
		//}

		//
		//
		
		//Vector2 direction = mousePos - Vector2(9, 9);
		//direction.normalize();
		//
		//auto list = tree.raycast({ 9, 9 }, direction);
		//

		//RendererQtImpl::renderPoint(&painter, &camera, { 9, 9 }, pen3);
		//RendererQtImpl::renderPoint(&painter, &camera, mousePos, pen3);
		//RendererQtImpl::renderLine(&painter, &camera, { 9, 9 }, mousePos, pen2);
		//

		//for (auto elem : list)
		//{
		//	ShapePrimitive p1;
		//	p1.rotation = elem->rotation();
		//	p1.shape = elem->shape();
		//	p1.transform = elem->position();

		//	RendererQtImpl::renderShape(&painter, &camera, p1, pen3);
		//}

		//for(auto iter = contactMaintainer.m_contactTable.begin(); iter != contactMaintainer.m_contactTable.end(); ++iter)
		//{
		//	for(auto& elem:iter->second)
		//	{
		//		RendererQtImpl::renderPoint(&painter, &camera, elem.bodyA->toWorldPoint(elem.localA), penApproximate4);
		//		RendererQtImpl::renderPoint(&painter, &camera, elem.bodyB->toWorldPoint(elem.localB), penApproximate3);
		//	}
		//}

		//auto potentialList = tree.generate();
		//for (auto pair : potentialList)
		//{
		//	ShapePrimitive shapeA, shapeB;
		//	shapeA.shape = pair.first->shape();
		//	shapeA.rotation = pair.first->rotation();
		//	shapeA.transform = pair.first->position();

		//	shapeB.shape = pair.second->shape();
		//	shapeB.rotation = pair.second->rotation();
		//	shapeB.transform = pair.second->position();

		//	auto [isColliding, simplex] = GJK::gjk(shapeA, shapeB);
		//	if (isColliding)
		//	{
		//		simplex = GJK::epa(shapeA, shapeB, simplex);
		//		PenetrationSource source = GJK::dumpSource(simplex);

		//		const auto info = GJK::dumpInfo(source);
		//		auto [clipEdgeA, clipEdgeB] = ContactGenerator::recognize(shapeA, shapeB, info.normal);
		//		auto pairList = ContactGenerator::clip(clipEdgeA, clipEdgeB, info.normal);

		//		RendererQtImpl::renderLine(&painter, &camera, clipEdgeA.p1, clipEdgeA.p2, penApproximate1);
		//		RendererQtImpl::renderLine(&painter, &camera, clipEdgeB.p1, clipEdgeB.p2, penApproximate2);

		//		for(auto& elem: pairList)
		//		{
		//			RendererQtImpl::renderPoint(&painter, &camera, elem.pointA, penApproximate3);
		//			RendererQtImpl::renderPoint(&painter, &camera, elem.pointB, penApproximate4);
		//		}
		//	}
		//}

		//ShapePrimitive p1, p2;
		//p1.rotation = rect->rotation();
		//p1.shape = rect->shape();
		//p1.transform = rect->position();


		//p2.rotation = rect2->rotation();
		//p2.shape = rect2->shape();
		//p2.transform = rect2->position();
		//auto result = GJK::distance(p1, p2);
		//RendererQtImpl::renderLine(&painter, &camera, result.pointA, result.pointB, pen3);
		//
		//
		//auto result = Detector::detect(rect, rect2);
		//if(result.isColliding)
		//{

		//	//ShapePrimitive primitive;
		//	//primitive.shape = rect->shape();
		//	//primitive.rotation = rect->rotation();
		//	//primitive.transform = rect->position();
		//	//RendererQtImpl::renderShape(&painter, &camera, primitive, pen2);


		//	//primitive.shape = rect2->shape();
		//	//primitive.rotation = rect2->rotation();
		//	//primitive.transform = rect2->position();
		//	//RendererQtImpl::renderShape(&painter, &camera, primitive, pen2);

		//	RendererQtImpl::renderLine(&painter, &camera, result.contactList[0].pointA, result.contactList[0].pointB, pen2);
		//}
		
		//QPen pen(Qt::cyan, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
		//auto result = CCD::query(dbvh.root(), rect2, dt);
		////fmt::print("toi exist:{}\n", result.has_value());
		//if (result.has_value())
		//{
		//	auto list = result.value();
		//	for(auto& pair: list)
		//	{
		//		fmt::print("toi:{}\n", pair.toi);
		//		Body::PhysicsAttribute origin = rect2->physicsAttribute();
		//		rect2->stepPosition(pair.toi);
		//		ShapePrimitive primitive;
		//		primitive.shape = rect2->shape();
		//		primitive.rotation = rect2->rotation();
		//		primitive.transform = rect2->position();
		//		RendererQtImpl::renderShape(&painter, &camera, primitive, pen2);
		//		rect2->setPhysicsAttribute(origin);
		//	}
		//}
	}

	void Window::resizeEvent(QResizeEvent* e)
	{

		camera.setViewport({ {0, 0}, {e->size().width() - camera.viewport().topLeft.x,
									   e->size().height() - camera.viewport().topLeft.y } });
		
		this->repaint();
	}

	void Window::mousePressEvent(QMouseEvent* e)
	{
		Vector2 pos(e->pos().x(), e->pos().y());
		mousePos = camera.screenToWorld(pos);
		if (e->button() == Qt::RightButton)
		{
			cameraTransform = true;
		}
		if (mj == nullptr)
			return;
		for(auto& body: m_world.bodyList())
		{
			Vector2 point = mousePos - body->position();
			point = Matrix2x2(-body->rotation()).multiply(point);
			if(body->shape()->contains(point) && selectedBody == nullptr)
			{
				selectedBody = body.get();

				auto prim = mj->primitive();
				prim.localPointA = body->toLocalPoint(mousePos);
				prim.bodyA = body.get();
				prim.targetPoint = mousePos;
				mj->setActive(true);
				mj->set(prim);
				break;
			}
		}
	}

	void Window::mouseReleaseEvent(QMouseEvent* e)
	{
		Vector2 pos(e->pos().x(), e->pos().y());
		mousePos = camera.screenToWorld(pos);

		if (mj == nullptr)
			return;
		mj->setActive(false);
		clickPos.clear();
		cameraTransform = false;
		selectedBody = nullptr;
	}


	void Window::mouseMoveEvent(QMouseEvent* e)
	{
		//testHit(e->pos());

		Vector2 pos(e->pos().x(), e->pos().y());
		//originPoint.set(m_world.screenToWorld(pos));
		//mousePrim.mousePoint = mousePos;
		Vector2 tf = camera.screenToWorld(pos) - mousePos;
		if(cameraTransform)
		{
			tf *= camera.meterToPixel();
			camera.setTransform(camera.transform() + tf);
		}
		if(selectedBody != nullptr)
		{
			//selectedBody->position() += tf;
		}
		mousePos = camera.screenToWorld(pos);

		if (mj == nullptr)
			return;
		auto prim = mj->primitive();
		prim.targetPoint = mousePos;
		mj->set(prim);
		repaint();
	}

	void Window::mouseDoubleClickEvent(QMouseEvent* event)
	{
		Vector2 pos(event->pos().x(), event->pos().y());
		mousePos = camera.screenToWorld(pos);
        //fmt::print("select body at {}\n", clickPos);
		for (auto& body : m_world.bodyList())
		{
			Vector2 point = mousePos - body->position();
			point = Matrix2x2(-body->rotation()).multiply(point);
			if (body->shape()->contains(point) && selectedBody == nullptr)
			{
				//tree.remove(body.get());
				//m_world.removeBody(body.get());
				break;
			}
		}
	}

	void Window::keyPressEvent(QKeyEvent* event)
	{
		switch (event->key())
		{
		case Qt::Key_J:
		{
			camera.setJointVisible(!camera.jointVisible());
			break;
		}
		case Qt::Key_B:
		{
			camera.setBodyVisible(!camera.bodyVisible());
			break;
		}
		case Qt::Key_D:
		{
			camera.setDbvhVisible(!camera.dbvhVisible());
			break;
		}
		case Qt::Key_A:
		{
			camera.setAabbVisible(!camera.aabbVisible());
			break;
		}
		case Qt::Key_T:
		{
			camera.setTreeVisible(!camera.treeVisible());
			break;
		}
		case Qt::Key_X:
		{
			camera.setAxisVisible(!camera.axisVisible());
			break;
		}
		case Qt::Key_G:
		{
			camera.setGridScaleLineVisible(!camera.gridScaleLineVisible());
			break;
		}
		case Qt::Key_Space:
		{
			isStop = !isStop;
			break;
		}
		case Qt::Key_R:
		{
			camera.setRotationLineVisible(!camera.rotationLineVisible());
			break;
		}
		case Qt::Key_L:
		{
			if (camera.targetBody() != nullptr)
				camera.setTargetBody(nullptr);
			break;
		}
		case Qt::Key_C:
		{
			camera.setCenterVisible(!camera.centerVisible());
			break;
		}
		default:
			break;
		}
		repaint();
	}

	void Window::keyReleaseEvent(QKeyEvent* event)
	{

		repaint();
	}

	void Window::wheelEvent(QWheelEvent* event)
	{
		//fmt::print("delta:{},{}\n", event->angleDelta().x(), event->angleDelta().y());
		if (event->angleDelta().y() > 0)
			camera.setMeterToPixel(camera.meterToPixel() + camera.meterToPixel() / 4.0);
		else
			camera.setMeterToPixel(camera.meterToPixel() - camera.meterToPixel() / 4.0);
		repaint();
	}
	

	void Window::createStackBox(const uint16_t& row = 10, const real& margin = 65, const real& spacing = 55)
	{
		for (real j = row; j > 0; j--)
		{
			for (real i = 0; i < 2 * (row - j) + 1; i++)
			{
				Body* body = m_world.createBody();
				body->position().set({
					(-spacing * (row - j) + i * spacing), j * margin + margin - 6
				});
				body->setShape(rectangle_ptr);
				body->rotation() = 0;
				body->setMass(400);
				body->setType(Body::BodyType::Dynamic);
				dbvh.insert(body);
			}
		}


		ground = m_world.createBody();
		ground->setShape(land_ptr);
		ground->position().set({0, -8});
		ground->setMass(Constant::Max);
		ground->setType(Body::BodyType::Static);
		dbvh.insert(ground);
	}

	void Window::createBoxesAndGround(const real& count)
	{
		for (real j = 0; j < count; j+=1.0f)
		{
			for(real i = 0;i < count; i+=1.0f)
			{
				Body* body = m_world.createBody();
				body->position().set({ i - 15.0f, j * 1.1f + 2.0f});
				body->setShape(rectangle_ptr);
				body->rotation() = 0.0f;
				body->setMass(1.0f);
				body->setType(Body::BodyType::Dynamic);
				body->setFriction(0.8f);
				body->setRestitution(0.0f);
				camera.setTargetBody(body);
				tree.insert(body);
			}
		}

		//ground = m_world.createBody();
		//ground->setShape(edge_ptr);
		//ground->position().set({0, -1.0});
		//ground->setMass(Constant::Max);
		//ground->setType(Body::BodyType::Static);
		//camera.setTargetBody(ground);
		//camera.setMeterToPixel(120);
		//dbvh.insert(ground);
	}
}
