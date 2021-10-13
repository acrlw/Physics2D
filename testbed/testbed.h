#ifndef PHYSICS2D_TESTBED_H
#define PHYSICS2D_TESTBED_H
#include "include/physics2d.h"
#include "QApplication"
#include "testbed/window.h"


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

	private:
		void clearAll();

		bool m_isStop = true;
		bool m_cameraViewportMovement = false;

		PhysicsWorld m_world;
		ContactMaintainer m_maintainer;
		Tree m_tree;
		DBVH m_dbvh;
		Body* m_selectedBody;

		Utils::Camera m_camera;

		PointJoint* m_mouseJoint;
		PointJointPrimitive m_pointJointPrimitive;

		QTimer m_worldTimer;
		QTimer m_painterTimer;

		Frame* m_currentFrame;

		Vector2 m_mousePos;
	};

	
}
#endif
