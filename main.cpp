#include "testbed/testbed.h"

int main(int argc, char* argv[])
{

	QApplication app(argc, argv);
	Physics2D::TestBed testbed;
	testbed.show();
	return app.exec();
}