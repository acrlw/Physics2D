#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPainter>
#include <QPaintEvent>
#include <QDebug>
#include <QTimer>
#include <QTime>
#include <QMouseEvent>
#include <QKeyEvent>
#include <albody.h>
#include <alrenderer.h>
#include <alcollision.h>
#include <alworld.h>
#include <QSlider>
QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
public slots:
    void processObjectMovement();

protected:
    void paintEvent(QPaintEvent *) override;
    void resizeEvent(QResizeEvent *e) override;
    void mousePressEvent(QMouseEvent *) override;
    void mouseReleaseEvent(QMouseEvent *e) override;
    void mouseMoveEvent(QMouseEvent *e) override;
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;
private:
    Ui::MainWindow *ui;


    //alCircleCircleCollisionDetector cccd1;
//    alMeasurer m_measurer;

    QTimer m_timer;
    QPointF m_mousePos;
    bool m_drag = false;
    bool m_drag2 = false;
    bool m_mousePress = false;
    QPointF m_mouseStart;
    alPolygon m_polygon;
    alPolygon m_polygon2;

    alPolygonRenderer m_polygonRenderer;
    alCircleRenderer m_circleRenderer;
    alEllipseRenderer m_ellipseRenderer;
    alRectangleRenderer m_rectangleRenderer;
    alEdgeRenderer m_edgeRenderer;
    alCurveRenderer m_curveRenderer;

    alEdge m_edgeList[2];
    alCurveEdge m_curveList[2];
    alRectangle m_rect;
    alEllipse m_ellipse, m_ellipse2;
    
    alWorld m_world;

    float m_step1, m_step2;
    QPointF m_control1, m_control2;
    QPointF m_start, m_end;
    bool m_A, m_S, m_D, m_W;
    bool m_CCW, m_ACCW;
};
#endif // MAINWINDOW_H
