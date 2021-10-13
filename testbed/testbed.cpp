#include "testbed/testbed.h"

namespace Physics2D
{
	TestBed::TestBed(QWidget* parent)
	{
		this->setWindowTitle("Testbed");
		this->resize(1920, 1080);
		this->setMouseTracking(true);

		this->setFont(QFont("Consolas", 10));

		createControlPanel();

		m_world.setEnableGravity(true);
		m_world.setGravity({ 0, -9.8f });
		m_world.setLinearVelocityDamping(0.1f);
		m_world.setAirFrictionCoefficient(0.8f);
		m_world.setAngularVelocityDamping(0.1f);
		m_world.setEnableDamping(true);
		m_world.setPositionIteration(8);
		m_world.setVelocityIteration(6);

		m_pointJointPrimitive.bodyA = nullptr;
		m_mouseJoint = m_world.createJoint(m_pointJointPrimitive);
		m_mouseJoint->setActive(false);

		m_camera.setViewport(Utils::Camera::Viewport(Vector2(0, 0), Vector2(1920, 1080)));
		m_camera.setWorld(&m_world);
		m_camera.setDbvh(&m_dbvh);
		m_camera.setTree(&m_tree);
		m_camera.setContactMaintainer(&m_maintainer);

		m_camera.setAabbVisible(false);
		m_camera.setDbvhVisible(false);
		m_camera.setTreeVisible(false);
		m_camera.setAxisVisible(false);
		m_camera.setGridScaleLineVisible(false);


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
		for (auto& elem : m_world.bodyList())
			m_tree.update(elem.get());

		if(m_isStop)
			return;

		m_world.stepVelocity(dt);

		auto potentialList = m_tree.generate();
		for (auto pair : potentialList)
		{
			auto result = Detector::detect(pair.first, pair.second);
			if (result.isColliding)
				m_maintainer.add(result);
		}
		m_maintainer.clearInactivePoints();
		m_world.prepareVelocityConstraint(dt);

		for (int i = 0; i < m_world.velocityIteration(); ++i)
		{
			m_world.solveVelocityConstraint(dt);
			m_maintainer.solveVelocity(dt);
		}

		m_world.stepPosition(dt);

		for (int i = 0; i < m_world.positionIteration(); ++i)
		{
			m_maintainer.solvePosition(dt);
			m_world.solvePositionConstraint(dt);
		}
		m_maintainer.deactivateAllPoints();
		
	}

	void TestBed::changeFrame(const QString& text)
	{
		clearAll();
		switch (qobject_cast<QComboBox*>(sender())->currentIndex())
		{
		case 0:
			m_currentFrame = new BitmaskFrame(&m_world, &m_maintainer, &m_tree, &m_dbvh);
			break;
		case 1:
			m_currentFrame = new BridgeFrame(&m_world, &m_maintainer, &m_tree, &m_dbvh);
			break;
		case 2:
			m_currentFrame = new BroadPhaseFrame(&m_world, &m_maintainer, &m_tree, &m_dbvh);
			break;
		case 3:
			m_currentFrame = new ChainFrame(&m_world, &m_maintainer, &m_tree, &m_dbvh);
			break;
		case 4:
			m_currentFrame = new CollisionFrame(&m_world, &m_maintainer, &m_tree, &m_dbvh);
			break;
		case 5:
			m_currentFrame = new DominoFrame(&m_world, &m_maintainer, &m_tree, &m_dbvh);
			break;
		case 6:
			m_currentFrame = new FrictionFrame(&m_world, &m_maintainer, &m_tree, &m_dbvh);
			break;
		case 7:
			m_currentFrame = new GeometryFrame(&m_world, &m_maintainer, &m_tree, &m_dbvh);
			break;
		case 8:
			m_currentFrame = new JointsFrame(&m_world, &m_maintainer, &m_tree, &m_dbvh);
			break;
		case 9:
			m_currentFrame = new NarrowphaseFrame(&m_world, &m_maintainer, &m_tree, &m_dbvh);
			break;
		case 10:
			m_currentFrame = new RaycastFrame(&m_world, &m_maintainer, &m_tree, &m_dbvh);
			break;
		case 11:
			m_currentFrame = new RestitutionFrame(&m_world, &m_maintainer, &m_tree, &m_dbvh);
			break;
		case 12:
			m_currentFrame = new SensorFrame(&m_world, &m_maintainer, &m_tree, &m_dbvh);
			break;
		case 13:
			m_currentFrame = new StackingFrame(&m_world, &m_maintainer, &m_tree, &m_dbvh);
			break;
		case 14:
			m_currentFrame = new WreckingBallFrame(&m_world, &m_maintainer, &m_tree, &m_dbvh);
			break;
			default:
				break;
		}
		if(m_currentFrame != nullptr)
			m_currentFrame->load();
	}

	void TestBed::clearAll()
	{
		m_world.clearAllBodies();
		m_world.clearAllJoints();
		m_maintainer.clearAll();
		m_tree.clearAll();
		m_dbvh.cleanUp(m_dbvh.root());
		m_pointJointPrimitive.bodyA = nullptr;
		m_mouseJoint = m_world.createJoint(m_pointJointPrimitive);
		m_mouseJoint->setActive(false);
		if(m_currentFrame != nullptr)
		{
			m_currentFrame->release();
			delete m_currentFrame;
			m_currentFrame = nullptr;
		}
	}

	void TestBed::createControlPanel()
	{
		m_controlPanel = new QWidget(this);

		QPalette palette;
		palette.setColor(QPalette::WindowText, Qt::green);
		QVBoxLayout* mainLayout = new QVBoxLayout;
		QComboBox* scenes = new QComboBox;
		QStringList items;
		items << "Bitmask" << "Bridge" << "Broadphase" << "Chain" << "Collision" << "Domino" << "Friction" <<
			"Geometry" << "Joints" << "Narrowphase" << "Raycast" << "Restitution" << "Sensor" << "Stacking" <<
			"Wrecking Ball";
		scenes->addItems(items);
		connect(scenes, &QComboBox::currentTextChanged, this, &TestBed::changeFrame);


		QSlider* posIter = new QSlider(Qt::Horizontal);
		QSlider* velIter = new QSlider(Qt::Horizontal);
		QSlider* deltaTime = new QSlider(Qt::Horizontal);
		QFormLayout* formLayout = new QFormLayout;
		QLabel* lblPosIter = new QLabel("Position Iterations: ");
		QLabel* lblVelIter = new QLabel("Velocity Iterations: ");
		QLabel* lblDtIter = new QLabel("Delta Time: ");
		formLayout->addRow(new QLabel("Scenes: "), scenes);
		formLayout->addRow(lblPosIter, posIter);
		formLayout->addRow(lblVelIter, velIter);
		formLayout->addRow(lblDtIter, deltaTime);

		posIter->setTracking(true);
		velIter->setTracking(true);
		deltaTime->setTracking(true);

		posIter->setRange(1, 20);
		velIter->setRange(1, 20);
		deltaTime->setRange(1, 120);

		connect(posIter, &QSlider::valueChanged, this, [=](int value)
			{
				m_world.setPositionIteration(value);
				lblPosIter->setText("Position Iterations: " + QString::number(value));
			});
		connect(velIter, &QSlider::valueChanged, this, [=](int value)
			{
				m_world.setVelocityIteration(value);
				lblVelIter->setText("Velocity Iterations: " + QString::number(value));
			});
		connect(deltaTime, &QSlider::valueChanged, this, [=](int value)
			{
				dt = 1.0f / static_cast<real>(value);
				lblDtIter->setText("Delta Time: " + QString::number(value) + " Hz");
			});


		posIter->setValue(8);
		velIter->setValue(6);
		deltaTime->setValue(60);


		QGroupBox* groupBox = new QGroupBox("Scenes And Sliders");
		groupBox->setPalette(palette);
		groupBox->setLayout(formLayout);

		QVBoxLayout* vbSwitchers = new QVBoxLayout;
		QCheckBox* cbBodyVisible = new QCheckBox("Body Visible");
		QCheckBox* cbAABBVisible = new QCheckBox("AABB Visible");
		QCheckBox* cbJointVisible = new QCheckBox("Joint Visible");
		QCheckBox* cbCenterVisible = new QCheckBox("Center Visible");
		QCheckBox* cbAngleVisible = new QCheckBox("Angle Visible");
		QCheckBox* cbGridVisible = new QCheckBox("Grid Scale Line Visible");
		QCheckBox* cbTreeVisible = new QCheckBox("Tree Visible");
		QCheckBox* cbContactVisible = new QCheckBox("Contacts Visible");
		QCheckBox* cbAxisVisible = new QCheckBox("Axis Visible");
		QCheckBox* cbUserDrawVisible = new QCheckBox("User Draw Visible");
		QCheckBox* cbStop = new QCheckBox("Stop");
		cbStop->setChecked(true);

		connect(cbGridVisible, &QCheckBox::stateChanged, this, [=](int state)
			{
				m_camera.setGridScaleLineVisible(state);
			});
		connect(cbBodyVisible, &QCheckBox::stateChanged, this, [=](int state)
			{
				m_camera.setBodyVisible(state);
			});
		connect(cbJointVisible, &QCheckBox::stateChanged, this, [=](int state)
			{
				m_camera.setJointVisible(state);
			});
		connect(cbTreeVisible, &QCheckBox::stateChanged, this, [=](int state)
			{
				m_camera.setTreeVisible(state);
			});
		connect(cbContactVisible, &QCheckBox::stateChanged, this, [=](int state)
			{
				m_camera.setContactVisible(state);
			});
		connect(cbAxisVisible, &QCheckBox::stateChanged, this, [=](int state)
			{
				m_camera.setAxisVisible(state);
			});
		connect(cbUserDrawVisible, &QCheckBox::stateChanged, this, [=](int state)
			{
				m_userDraw = state;
			});
		connect(cbCenterVisible, &QCheckBox::stateChanged, this, [=](int state)
			{
				m_camera.setCenterVisible(state);
			});
		connect(cbAngleVisible, &QCheckBox::stateChanged, this, [=](int state)
			{
				m_camera.setRotationLineVisible(state);
			});
		connect(cbStop, &QCheckBox::stateChanged, this, [=](int state)
			{
				m_isStop = state;
			});


		vbSwitchers->addWidget(cbBodyVisible);
		vbSwitchers->addWidget(cbAABBVisible);
		vbSwitchers->addWidget(cbJointVisible);
		vbSwitchers->addWidget(cbGridVisible);
		vbSwitchers->addWidget(cbTreeVisible);
		vbSwitchers->addWidget(cbContactVisible);
		vbSwitchers->addWidget(cbAxisVisible);
		vbSwitchers->addWidget(cbUserDrawVisible);
		vbSwitchers->addWidget(cbAngleVisible);
		vbSwitchers->addWidget(cbCenterVisible);
		vbSwitchers->addWidget(cbStop);

		QGroupBox* groupSwitchers = new QGroupBox("Switchers");
		groupSwitchers->setPalette(palette);
		groupSwitchers->setLayout(vbSwitchers);


		mainLayout->addWidget(groupBox);
		mainLayout->addWidget(groupSwitchers);

		m_controlPanel->setLayout(mainLayout);
		m_controlPanel->setPalette(palette);

		m_controlPanel->show();

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
