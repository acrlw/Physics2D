#include "mainwindow.h"
#include "ui_mainwindow.h"
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    m_polygon.addVertex(alVector2(0, 6));
    m_polygon.addVertex(alVector2(-1, 6));
    m_polygon.addVertex(alVector2(-2, 5));
    m_polygon.addVertex(alVector2(-3, 4));
    m_polygon.addVertex(alVector2(-4, 3));
    m_polygon.addVertex(alVector2(-5, 2));
    m_polygon.addVertex(alVector2(-6, 1));
    m_polygon.addVertex(alVector2(-5, -1));
    m_polygon.addVertex(alVector2(-4, -2));
    m_polygon.addVertex(alVector2(-3, -3));
    m_polygon.addVertex(alVector2(-2, -4));
    m_polygon.addVertex(alVector2(-1, -5));
    m_polygon.addVertex(alVector2(0, -6));
    m_polygon.addVertex(alVector2(1, -5));
    m_polygon.addVertex(alVector2(2, -4));
    m_polygon.addVertex(alVector2(3, -3));
    m_polygon.addVertex(alVector2(4, -2));
    m_polygon.addVertex(alVector2(5, -1));
    m_polygon.addVertex(alVector2(6, 0));
    m_polygon.addVertex(alVector2(5, 1));
    m_polygon.addVertex(alVector2(4, 2));
    m_polygon.addVertex(alVector2(3, 3));
    m_polygon.addVertex(alVector2(2, 4));
    m_polygon.addVertex(alVector2(1, 5));
    m_polygon.addVertex(alVector2(0, 6));
    m_polygon.scale(10);
    m_polygon.position().set(300, 300);
    m_polygon.setAngle(45);

    m_polygonRenderer.setThickness(2);
    m_polygonRenderer.polygonList().append(&m_polygon);
	
    m_step1 = 1.0 / 60.0f;
    m_step2 = 1.0 / 60.0f;
    connect(&m_timer,&QTimer::timeout,this,&MainWindow::processObjectMovement);
    m_timer.setInterval(alTimerInterval);
    m_timer.start();
    this->resize(1920, 1080);

    m_edgeList[0].set(alVector2(2, 6), alVector2(30, 0));
    m_edgeList[1].set(alVector2(30, 0), alVector2(60, 6));
    m_edgeList[0].scale(12);
    m_edgeList[1].scale(12);
    m_edgeList[0].translate(104, 700);
    m_edgeList[1].translate(104, 700);


    m_edgeRenderer.setThickness(2);
    m_edgeRenderer.edgeList().append(&m_edgeList[0]);
    m_edgeRenderer.edgeList().append(&m_edgeList[1]);


    m_curveList[0].set(alVector2(2, 6), alVector2(25, 25), alVector2(35, 9), alVector2(40, 4));
    m_curveList[1].set(alVector2(40, 4), alVector2(55, -15), alVector2(65, 25), alVector2(85, -2));
    m_curveList[0].scale(12);
    m_curveList[1].scale(12);
    m_curveList[0].translate(800, 700);
    m_curveList[1].translate(800, 700);

    m_curveRenderer.setThickness(2);
    m_curveRenderer.edgeList().append(&m_curveList[0]);
    m_curveRenderer.edgeList().append(&m_curveList[1]);

    m_rect.setWidth(6);
    m_rect.setHeight(8);
    m_rect.scale(20);
    m_rect.position().set(1200, 20);
    m_rect.setAngle(45);
    m_rectangleRenderer.setThickness(2);
    m_rectangleRenderer.rectangleList().append(&m_rect);

    m_ellipse.setTopLeft(alVector2(-6, 8));
    m_ellipse.setBottomRight(alVector2(6, -8));
    //m_ellipse2.setTopLeft(alVector2(-3, 4));
    //m_ellipse2.setBottomRight(alVector2(3, -4));
    //m_ellipse2.scale(25);
    m_ellipse.scale(15);
    m_ellipse.position().set(1600, 600);
    //m_ellipse2.position().set(400, 800);
    m_ellipseRenderer.setThickness(2);
    m_ellipseRenderer.ellipseList().append(&m_ellipse);
    m_ellipseRenderer.ellipseList().append(&m_ellipse2);

    m_A = m_S = m_D = m_W = false;
    m_CCW = m_ACCW = false;

//    m_start.setX(10);
//    m_start.setY(10);
//    m_end.setX(700);
//    m_end.setY(-120);
//    m_control1.setX(250);
//    m_control1.setY(200);

//    m_control2.setX(450);
//    m_control2.setY(-300);
}
void MainWindow::processObjectMovement()
{


    QPainter painter;


    m_world.step(m_step2, m_step1);
    m_step1 = m_step2;
    repaint();
}
void MainWindow::paintEvent(QPaintEvent *e)
{

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    //    m_rectangleRenderer.render(&painter);
    //    m_measurer.render(&painter);
    //uint32_t circleTouch = 0;
    if(m_mousePress)
    {
        QPen st(Qt::blue, 6, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        painter.setPen(st);
        painter.drawPoint(m_mouseStart);
        QPen ed(Qt::blue, 6, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        painter.setPen(ed);
        painter.drawPoint(m_mousePos);
        QPen line(Qt::darkCyan, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        painter.setPen(line);
        painter.drawLine(m_mouseStart, m_mousePos);
    }
    if (m_A)
        m_ellipse.position().setX(m_ellipse.position().x() - 6);
    if (m_S)
        m_ellipse.position().setY(m_ellipse.position().y() + 6);
    if (m_D)
        m_ellipse.position().setX(m_ellipse.position().x() + 6);
    if (m_W)
        m_ellipse.position().setY(m_ellipse.position().y() - 6);

    if (m_CCW)
        m_ellipse.setAngle(m_ellipse.angle() + 5);

    if (m_ACCW)
        m_ellipse.setAngle(m_ellipse.angle() - 5);
    alGJKCollisionDetector gjk;
    ContactInfo result;

    alVector2 min, max;
    foreach(ContactInfo info, gjk.curveDetection(&painter, &m_ellipse, &m_curveList[0]))
    {
        if (min.lengthSquare() == 0) {
            min = info.getPenetrationVector();
            continue;
        }
        else
        {
            if (min.lengthSquare() > info.getPenetrationVector().lengthSquare())
            {
                min = info.getPenetrationVector();
            }
        }
        if (max.lengthSquare() == 0) {
            max = info.getPenetrationVector();
            continue;
        }
        else
        {
            if (max.lengthSquare() < info.getPenetrationVector().lengthSquare())
            {
                max = info.getPenetrationVector();
            }
        }
    }
    //m_ellipse.position() += (min + max) / 2;
    foreach(ContactInfo info, gjk.curveDetection(&painter, &m_ellipse, &m_curveList[1]))
    {
        if (min.lengthSquare() == 0) {
            min = info.getPenetrationVector();
            continue;
        }
        else
        {
            if (min.lengthSquare() < info.getPenetrationVector().lengthSquare())
            {
                min = info.getPenetrationVector();
            }
        }
        if (max.lengthSquare() == 0) {
            max = info.getPenetrationVector();
            continue;
        }
        else
        {
            if (max.lengthSquare() < info.getPenetrationVector().lengthSquare())
            {
                max = info.getPenetrationVector();
            }
        }
    }
    //m_ellipse.position() += (min + max) / 2;
    result = gjk.detect(&painter, &m_ellipse, &m_edgeList[0]);
    if (result.getIsCollide())
    {
        //m_ellipse.position() += result.getPenetrationVector();
    }
    result = gjk.detect(&painter, &m_ellipse, &m_edgeList[1]);
    if (result.getIsCollide())
    {
        //m_ellipse.position() += result.getPenetrationVector();
    }
    result = gjk.detect(&painter, &m_polygon, &m_rect);
//    QPen st(Qt::darkGreen, 4, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
//    QPen p(Qt::gray, 10, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
//    QPen p2(Qt::gray, 6, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
//    QPen l(Qt::darkCyan, 4, Qt::DotLine, Qt::RoundCap, Qt::RoundJoin);
//    painter.setPen(p);
//    QPolygonF points;
//    points << m_start << m_end << m_control1 << m_control2;
//    painter.translate(500, 500);
//    painter.drawPoints(points);
//    painter.setPen(l);
//    painter.drawLine(m_start, m_control1);
//    painter.drawLine(m_end, m_control2);
//    painter.setPen(st);
//    //QPainterPath path;
//    //path.cubicTo(m_control1, m_control2, m_end);
//    //painter.drawPath(path);
//    painter.setPen(p2);

//    for(float t = 0.01, d = 0.03; t < 1.0f; t += 0.05, d = t + 0.02)
//    {
//        QPointF target1 =
//                pow((1.0f - t), 3) * m_start + 3 * t * pow(1.0f - t, 2) * m_control1 + 3 * pow(t, 2) * (1.0f - t) * m_control2 + pow(t, 3) * m_end;

////        QPointF target2 =
////                pow((1.0f - d), 3) * m_start + 3 * d * pow(1.0f - d, 2) * m_control1 + 3 * pow(d, 2) * (1.0f - d) * m_control2 + pow(d, 3) * m_end;
////        painter.drawLine(target1, target2);
//        painter.drawPoint(target1);

//    }
    //painter.translate(-500, -500);


//    result = gjk.detect(&painter, &m_ellipse, &m_edgeList[0]);
//    result = gjk.detect(&painter, &m_ellipse, &m_edgeList[1]);
    //alGJKCollisionDetector gjk;
    //ContactInfo result = gjk.detect(&painter, &m_rect, &m_ellipse);



    m_polygonRenderer.render(&painter);
    m_circleRenderer.render(&painter);
    m_ellipseRenderer.render(&painter);
    m_rectangleRenderer.render(&painter);
    m_edgeRenderer.render(&painter);
    m_curveRenderer.render(&painter);

}
void MainWindow::mousePressEvent(QMouseEvent * e)
{
    m_mousePress = true;
    m_mouseStart = e->pos();
    qDebug() << "press";
    QPointF d = m_rect.position().toPointF() - e->pos();
    if(d.manhattanLength() < 80)
    {
        m_rect.position().setX(e->pos().x());
        m_rect.position().setY(e->pos().y());
    }
    d = m_ellipse.position().toPointF() - e->pos();
    if(d.manhattanLength() < 80)
    {
        m_ellipse.position().setX(e->pos().x());
        m_ellipse.position().setY(e->pos().y());
    }
    QPointF di1 = m_control1 - e->pos() + QPoint(500, 500);
    QPointF di2 = m_control2 - e->pos() + QPoint(500, 500);
    if(di1.manhattanLength() < 10)
    {
        m_control1 = e->pos() - QPoint(500, 500);
    }
    if(di2.manhattanLength() < 10)
    {
        m_control2 = e->pos() - QPoint(500, 500);
    }
}
void MainWindow::mouseReleaseEvent(QMouseEvent *e)
{
    m_mousePress = false;
    m_mousePos = e->pos();
    QPointF d = m_rect.position().toPointF() - e->pos();
    if(d.manhattanLength() < 80)
    {
        m_rect.position().setX(e->pos().x());
        m_rect.position().setY(e->pos().y());
    }
    d = m_ellipse.position().toPointF() - e->pos();
    if(d.manhattanLength() < 80)
    {
        m_ellipse.position().setX(e->pos().x());
        m_ellipse.position().setY(e->pos().y());
    }
    qDebug() << "release";
    QPointF di1 = m_control1 - e->pos() + QPoint(500, 500);
    QPointF di2 = m_control2 - e->pos() + QPoint(500, 500);
    if(di1.manhattanLength() < 10)
    {
        m_control1 = e->pos() - QPoint(500, 500);
    }
    if(di2.manhattanLength() < 10)
    {
        m_control2 = e->pos() - QPoint(500, 500);
    }
}
void MainWindow::mouseMoveEvent(QMouseEvent *e)
{
    m_mousePos = e->pos();

    QPointF d = m_rect.position().toPointF() - e->pos();
    if(d.manhattanLength() < 80)
    {
        m_rect.position().setX(e->pos().x());
        m_rect.position().setY(e->pos().y());
    }
    d = m_ellipse.position().toPointF() - e->pos();
    if(d.manhattanLength() < 80)
    {
        m_ellipse.position().setX(e->pos().x());
        m_ellipse.position().setY(e->pos().y());
    }
    QPointF di1 = m_control1 - e->pos() + QPoint(500, 500);
    QPointF di2 = m_control2 - e->pos() + QPoint(500, 500);
    if(di1.manhattanLength() < 10)
    {
        m_control1 = e->pos() - QPoint(500, 500);
    }
    if(di2.manhattanLength() < 10)
    {
        m_control2 = e->pos() - QPoint(500, 500);
    }
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    switch (event->key()) {
    case Qt::Key_R:
    {
        m_CCW = true;
        break;
    }
    case Qt::Key_Q:
    {
        m_ACCW = true;
        break;
    }
    case Qt::Key_D:
    {
        m_D = true;
        break;
    }
    case Qt::Key_A:
    {

        m_A = true;
        break;
    }
    case Qt::Key_S:
    {

        m_S = true;
        break;
    }
    case Qt::Key_W:
    {
        m_W = true;
        break;
    }
    default:
        break;
    }
}

void MainWindow::keyReleaseEvent(QKeyEvent *event)
{
    switch (event->key()) {
    case Qt::Key_R:
    {
        m_CCW = false;
        break;
    }
    case Qt::Key_Q:
    {
        m_ACCW = false;
        break;
    }
    case Qt::Key_D:
    {
        m_D = false;
        break;
    }
    case Qt::Key_A:
    {
        m_A = false;
        break;
    }
    case Qt::Key_S:
    {
        m_S = false;
        break;
    }
    case Qt::Key_W:
    {
        m_W = false;
        break;
    }
    default:
        break;
    }
}
void MainWindow::resizeEvent(QResizeEvent *e)
{

}
MainWindow::~MainWindow()
{
    delete ui;
}

