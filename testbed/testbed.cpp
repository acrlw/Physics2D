#include "testbed/testbed.h"

namespace Physics2D
{
	TestBed::TestBed(QWidget* parent)
	{
		this->setWindowTitle("Testbed");
		this->resize(1920, 1080);
		this->setMouseTracking(true);

		m_world.setEnableGravity(true);
		m_world.setGravity({ 0, -9.8f });
		m_world.setLinearVelocityDamping(0.1f);
		m_world.setAirFrictionCoefficient(0.8f);
		m_world.setAngularVelocityDamping(0.1f);
		m_world.setEnableDamping(true);
		m_world.setPositionIteration(10);
		m_world.setVelocityIteration(8);

		m_pointJointPrimitive.bodyA = nullptr;
		m_mouseJoint = m_world.createJoint(m_pointJointPrimitive);
		m_mouseJoint->setActive(false);

		m_camera.setViewport(Utils::Camera::Viewport((0, 0), (1920, 1080)));
		m_camera.setWorld(&m_world);
		m_camera.setDbvh(&m_dbvh);
		m_camera.setTree(&m_tree);

		m_camera.setAabbVisible(false);
		m_camera.setDbvhVisible(false);
		m_camera.setTreeVisible(false);
		m_camera.setAxisVisible(true);
		m_camera.setGridScaleLineVisible(true);

		connect(&m_worldTimer, &QTimer::timeout, this, &TestBed::step);
		connect(&m_painterTimer, &QTimer::timeout, this, [=]
		{
				this->repaint();
		});


		m_worldTimer.setInterval(15);
		m_worldTimer.start();
		m_painterTimer.setInterval(15);
		m_painterTimer.start();

	}
	TestBed::~TestBed()
	{

	}
	void TestBed::step()
	{
	}

	void TestBed::clearAll()
	{

	}

	void TestBed::paintEvent(QPaintEvent*)
	{
		QPainter painter(this);
		m_camera.render(&painter);
	}
	void TestBed::resizeEvent(QResizeEvent* e)
	{
		m_camera.setViewport({ {0, 0}, {e->size().width() - m_camera.viewport().topLeft.x,
									   e->size().height() - m_camera.viewport().topLeft.y } });

		this->repaint();
	}
	void TestBed::mousePressEvent(QMouseEvent*e)
	{
		Vector2 pos(e->pos().x(), e->pos().y());
		m_mousePos = m_camera.screenToWorld(pos);
		if (e->button() == Qt::RightButton)
			m_cameraViewportMovement = true;

		if (m_mouseJoint == nullptr)
			return;

		for (auto& body : m_world.bodyList())
		{
			Vector2 point = m_mousePos - body->position();
			point = Matrix2x2(-body->rotation()).multiply(point);
			if (body->shape()->contains(point) && m_selectedBody == nullptr)
			{
				m_selectedBody = body.get();

				auto prim = m_mouseJoint->primitive();
				prim.localPointA = body->toLocalPoint(m_mousePos);
				prim.bodyA = body.get();
				prim.targetPoint = m_mousePos;
				m_mouseJoint->setActive(true);
				m_mouseJoint->set(prim);
				break;
			}
		}
	}
	void TestBed::mouseReleaseEvent(QMouseEvent* e)
	{
		Vector2 pos(e->pos().x(), e->pos().y());
		m_mousePos = m_camera.screenToWorld(pos);

		if (m_mouseJoint == nullptr)
			return;
		m_mouseJoint->setActive(false);

		m_cameraViewportMovement = false;
		m_selectedBody = nullptr;
	}
	void TestBed::mouseMoveEvent(QMouseEvent* e)
	{
		Vector2 pos(e->pos().x(), e->pos().y());
		
		Vector2 tf = m_camera.screenToWorld(pos) - m_mousePos;
		if (m_cameraViewportMovement)
		{
			tf *= m_camera.meterToPixel();
			m_camera.setTransform(m_camera.transform() + tf);
		}
		m_mousePos = m_camera.screenToWorld(pos);

		if (m_mouseJoint == nullptr)
			return;

		auto prim = m_mouseJoint->primitive();
		prim.targetPoint = m_mousePos;
		m_mouseJoint->set(prim);
	}
	void TestBed::mouseDoubleClickEvent(QMouseEvent* event)
	{
	}
	void TestBed::keyPressEvent(QKeyEvent* event)
	{
		switch (event->key())
		{
		case Qt::Key_J:
		{
			m_camera.setJointVisible(!m_camera.jointVisible());
			break;
		}
		case Qt::Key_B:
		{
			m_camera.setBodyVisible(!m_camera.bodyVisible());
			break;
		}
		case Qt::Key_D:
		{
			m_camera.setDbvhVisible(!m_camera.dbvhVisible());
			break;
		}
		case Qt::Key_A:
		{
			m_camera.setAabbVisible(!m_camera.aabbVisible());
			break;
		}
		case Qt::Key_T:
		{
			m_camera.setTreeVisible(!m_camera.treeVisible());
			break;
		}
		case Qt::Key_X:
		{
			m_camera.setAxisVisible(!m_camera.axisVisible());
			break;
		}
		case Qt::Key_G:
		{
			m_camera.setGridScaleLineVisible(!m_camera.gridScaleLineVisible());
			break;
		}
		case Qt::Key_Space:
		{
			m_isStop = !m_isStop;
			break;
		}
		case Qt::Key_R:
		{
			m_camera.setRotationLineVisible(!m_camera.rotationLineVisible());
			break;
		}
		case Qt::Key_L:
		{
			if (m_camera.targetBody() != nullptr)
				m_camera.setTargetBody(nullptr);
			break;
		}
		case Qt::Key_C:
		{
			m_camera.setCenterVisible(!m_camera.centerVisible());
			break;
		}
		default:
			break;
		}
	}
	void TestBed::keyReleaseEvent(QKeyEvent* event)
	{
	}
	void TestBed::wheelEvent(QWheelEvent* event)
	{
		if (event->angleDelta().y() > 0)
			m_camera.setMeterToPixel(m_camera.meterToPixel() + m_camera.meterToPixel() / 4.0);
		else
			m_camera.setMeterToPixel(m_camera.meterToPixel() - m_camera.meterToPixel() / 4.0);
		
	}
}
