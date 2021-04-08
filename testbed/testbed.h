#ifndef PHYSICS2D_TESTBED_H
#define PHYSICS2D_TESTBED_H
#include "include/physics2d.h"
#include "QApplication"
#include "QWidget"
#include "QPainter"
namespace Physics2D
{
	class TestBedApplication
	{
	public:
		TestBedApplication(int argc, char* argv[])
		{
			exec(argc, argv);
		}
		int exec(int argc, char* argv[])
		{
			QApplication app(argc, argv);
			
			return app.exec();
		}
	private:
	};
}
#endif
