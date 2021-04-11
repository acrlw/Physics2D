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
		void keyPressEvent(QKeyEvent* event) override;
		void keyReleaseEvent(QKeyEvent* event) override;
	
	private:
		void testShape(QPainter* painter);
		void createStackBox(const uint16_t& row, const uint16_t& margin, const uint16_t& spacing);
		void testHit(const QPoint& pos);
		void testAABB(QPainter* painter);
		void testBVH(QPainter* painter);
		World m_world;
		Rectangle rectangle;
		Ellipse ellipse;
		Edge edge;
		Curve curve;
		Polygon polygon;
		Circle circle;
		real m_angle = 0;
		const Body* m_lastBody = nullptr;
		QTimer m_timer;
	};
}
#endif