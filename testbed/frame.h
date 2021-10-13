#ifndef PHYSICS2D_TESTBED_FRAME_H
#define PHYSICS2D_TESTBED_FRAME_H

#include "include/physics2d.h"
#include "include/utils/camera.h"

#include <QMouseEvent>
#include <QKeyEvent>
#include <QWheelEvent>
#include <QPaintEvent>
#include <string>
namespace Physics2D
{
	class Frame
	{
	public:
		Frame(std::string name, PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh) : m_name(name), m_world(world), m_maintainer(maintainer),
		m_tree(tree), m_dbvh(dbvh){}
		virtual void load();
		virtual void release();
		virtual void render(QPainter* painter);
		virtual void onMousePress(QMouseEvent* event);
		virtual void onMouseRelease(QMouseEvent* event);
		virtual void onMouseMove(QMouseEvent* event);
		virtual void onMouseDoubleClick(QMouseEvent* event);
		virtual void onKeyPress(QKeyEvent* event);
		virtual void onKeyRelease(QKeyEvent* event);
		
		std::string name()const
		{
			return m_name;
		}
		void setName(const std::string& name)
		{
			m_name = name;
		}
	protected:
		std::string m_name;
		PhysicsWorld* m_world = nullptr;
		ContactMaintainer* m_maintainer = nullptr;
		Tree* m_tree = nullptr;
		DBVH* m_dbvh = nullptr;
	};
}

#endif // !PHYSICS2D_TESTBED_FRAME_H
