#ifndef PHYSICS2D_TESTBED_WINDOW_H
#define PHYSICS2D_TESTBED_WINDOW_H
#include <QPainter>
#include <QWindow>
#include <QWidget>
#include <QPen>
#include <QTimer>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QWheelEvent>
#include <QPaintEvent>
#include <QPainterPath>
#include <QRandomGenerator>
#include <include/physics2d.h>
#include <include/render/impl/renderer_qt.h>
#include <include/collision/algorithm/mpr.h>
#include <include/collision/algorithm/sat.h>
#include <include/collision/broadphase/dbvh.h>

#include "include/collision/algorithm/clip.h"
#include "include/collision/broadphase/tree.h"
#include "include/utils/camera.h"
#include "include/collision/continuous/ccd.h"

namespace Physics2D
{
	class Window : public QWidget
	{
		Q_OBJECT

	public:
		Window(QWidget* parent = nullptr);
		~Window();
	public slots:
		void process();

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
	
	private:
		void createPyramid();
		void testTree();
		void createBoxRoom();
		void createBoxesAndGround(const real& count = 10);
		void createStackBox(const uint16_t& row, const real& margin, const real& spacing);
		void testJoint();
		void testCollision();
		void createSnakeBody();
		void testDistanceJoint();
		void testPendulum();
		void testBroadphase();
		void testCCD();
		void testRaycast();
		void testCapsule();
		World m_world;
		Rectangle rectangle;
		Rectangle land;
		Ellipse ellipse;
		Edge edge;

		Edge boxHorizontal;
		Edge boxVertical;

		Curve curve;
		Polygon polygon;
		Circle circle;
		Capsule capsule;
		Sector sector;


		real m_angle = 0;
		Body* m_lastBody = nullptr;
		QTimer m_timer;
		Body* rect;
		Body* rect2;
		Body* rect3;
		Body* rect4;
		Body* ground;
		

		real angle = 0;
		real radius = 150;
		Vector2 originPoint = { 0, 0 };
		Vector2 targetPoint;
		std::vector<Vector2> m_rectCenter;
		Vector2 error;
		Vector2 lastError;
		Vector2 errorAll;
		Vector2 clickPos;
		Vector2 mousePos;
		DistanceJointPrimitive distancePrim;
		RotationJointPrimitive rotationPrim;
		OrientationJointPrimitive orientationPrim;
		PointJointPrimitive pointPrim;

		PointJoint* mj = nullptr;
		DBVH dbvh;
		int counter = 0;
		
		std::shared_ptr<Ellipse> ellipse_ptr;
		std::shared_ptr<Edge> edge_ptr;

		std::shared_ptr<Edge> horizontalWall;
		std::shared_ptr<Edge> verticalWall;

		std::shared_ptr<Curve> curve_ptr;
		std::shared_ptr<Polygon> polygon_ptr;
		std::shared_ptr<Rectangle> land_ptr;
		std::shared_ptr<Rectangle> rectangle_ptr;
		std::shared_ptr<Circle> circle_ptr;
		std::shared_ptr<Capsule> capsule_ptr;
		std::shared_ptr<Sector> sector_ptr;

		Utils::Camera camera;
		bool cameraTransform = false;
		bool isStop = false;
		Body* selectedBody = nullptr;

		Tree tree;
		real roomSize = 20;

		ContactMaintainer contactMaintainer;
		Integrator::SemiImplicitEuler<Vector2> euler;
		Integrator::VerletVelocity<Vector2> vv;
		Integrator::VerletPosition<Vector2> vp;
	};

	
}
#endif