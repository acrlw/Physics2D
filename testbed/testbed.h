#ifndef PHYSICS2D_TESTBED_H
#define PHYSICS2D_TESTBED_H
#include "include/physics2d.h"
#include "QApplication"
#include "testbed/window.h"


namespace Physics2D
{
	
	class TestBedApplication
	{
	public:
		TestBedApplication()
		{}
		static int exec(int argc, char* argv[])
		{
			QApplication app(argc, argv);
			Window window;
			window.show();
			return app.exec();
		}
	private:
	};
}
#endif
