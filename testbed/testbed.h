#ifndef PHYSICS2D_TESTBED_H
#define PHYSICS2D_TESTBED_H
#include "include/physics2d.h"
#include "QApplication"
#include "testbed/window.h"

#include "scenes/bitmask.h"
#include "scenes/bridge.h"
#include "scenes/broadphase.h"
#include "scenes/chain.h"
#include "scenes/collision.h"
#include "scenes/domino.h"
#include "scenes/friction.h"
#include "scenes/geometry.h"
#include "scenes/joints.h"
#include "scenes/narrowphase.h"
#include "scenes/raycast.h"
#include "scenes/restitution.h"
#include "scenes/sensor.h"
#include "scenes/stacking.h"
#include "scenes/wreckingball.h"

#include <QSlider>
#include <QVBoxLayout>
#include <QLabel>
#include <QComboBox>
#include <QCheckBox>
#include <QFormLayout>
#include <QGroupBox>

namespace Physics2D
{
	
	class TestBed : public QWidget
	{
		Q_OBJECT
	public:
		TestBed(QWidget* parent = nullptr);
		~TestBed();

	protected:
		void paintEvent(QPaintEvent*) override;
		void resizeEvent(QResizeEvent* e) override;
		void mousePressEvent(QMouseEvent*) override;
		void mouseReleaseEvent(QMouseEvent* e) override;
		void mouseMoveEvent(QMouseEvent* e) override;
		void mouseDoubleClickEvent(QMouseEvent* event) override;
		void keyPressEvent(QKeyEvent* event) override;
		void keyReleaseEvent(QKeyEvent* event) override;
		void wheelEvent(QWheelEvent* event)override;

	public slots:
		void step();
		void changeFrame(const QString& index);
	private:
		void clearAll();
		void createControlPanel();

		bool m_userDraw = false;
		bool m_isStop = true;
		bool m_cameraViewportMovement = false;

		QWidget* m_controlPanel;

		PhysicsWorld m_world;
		ContactMaintainer m_maintainer;
		Tree m_tree;
		DBVH m_dbvh;
		Body* m_selectedBody;
		PointJoint* m_mouseJoint;
		PointJointPrimitive m_pointJointPrimitive;
		

		Utils::Camera m_camera;


		QTimer m_worldTimer;
		QTimer m_painterTimer;

		Frame* m_currentFrame = nullptr;

		Vector2 m_mousePos;

		real dt = 1.0f / 60.0f;
	};

	
}
#endif
