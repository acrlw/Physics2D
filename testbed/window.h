#ifndef PHYSICS2D_TESTBED_WINDOW_H
#define PHYSICS2D_TESTBED_WINDOW_H
#include <QPainter>
#include <QWindow>
#include <QWidget>
#include <QPen>
#include <QTimer>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QPaintEvent>
#include <QPainterPath>
#include <include/physics2d.h>
#include <include/render/impl/renderer_qt.h>

#include "include/dynamics/constraint/constraint.h"
#include "include/dynamics/joint/joint.h"
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
	
	private:
		void createStackBox(const uint16_t& row, const real& margin, const real& spacing);
		void testHit(const QPoint& pos);
		void testAABB(QPainter* painter);
		void testBVH(QPainter* painter);
		void testCollision();
		void createSnakeBody();
		void testDistanceJoint();
		void testPendulum();
		World m_world;
		Rectangle rectangle;
		Rectangle land;
		Ellipse ellipse;
		Edge edge;
		Curve curve;
		Polygon polygon;
		Circle circle;
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
		Joint joint;
	};
}
#endif