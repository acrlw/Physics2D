#include "mainwindow.h"
#include "include/math/math.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    Physics2D::Matrix3x3 mat31(1, 2, 3, 4, 5, 6, 7, 8, 9);
    Physics2D::Matrix3x3 mat32(10, 13, 12, 21, 14, 15, 16, 17, 18);
    mat32.invert();
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
