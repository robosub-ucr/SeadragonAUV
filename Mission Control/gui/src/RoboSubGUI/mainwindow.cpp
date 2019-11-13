#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "compass.h"

#include <QPushButton>
#include <QLabel>
#include <QFrame>
#include <QTextBrowser>
#include <QTimer>
#include <QDateTime>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->pushButton->setText("System View");
    ui->pushButton_2->setText("Roscore");
    ui->pushButton_3->setText("AHRS");
    ui->pushButton_4->setText("TEST");
    ui->pushButton_5->setText("STM32");
    ui->pushButton_6->setText("Camera 1");
    ui->pushButton_7->setText("Camera 2");
    ui->pushButton_8->setText("State Machines");
    QTimer *timer = new QTimer(this);
    connect(timer , SIGNAL(timeout()) , this, SLOT(showTime()));
    timer->start();



}
void MainWindow::paintEvent(QPaintEvent *event){
    //draws compass
    //currently trying to figure out how to move this to a seperate class
    //to make it much more easily customizable
    //work in progress



    //creates a painter
    QPainter painter(this);

    //creates the compass
    Compass c(600,400);

    //sets fill to be blue
    painter.setBrush(Qt::blue);

    //sets the center to be 600, 400
    painter.translate(c.getCenterX(),c.getCenterY());

    //creates a brush, more on that later
    QBrush brush;

    //creates a pen, which dictates the color and style of the lines
    QPen pen;
    pen.setColor(Qt::white);
    pen.setWidth(5);

    //puts pen into painter
    painter.setPen(pen);

    //draws a rectagle from the top left corner (x,y, width, height)
    c.setSquareWidth(300);
    painter. drawRect(QRect(-c.getSquareWidth()/2, -c.getSquareWidth()/2, c.getSquareWidth(), c.getSquareWidth()));

    pen.setColor(Qt::black);
    painter.setBrush(Qt::black);

    //draws an ellepse from top left corner (x,y, center width, center height)
    painter.drawEllipse(-c.getSquareWidth()/2, -c.getSquareWidth()/2, c.getSquareWidth(), c.getSquareWidth());


    //defines a the points of a polygon
    c.setNeedleWidth(10);
    c.setNeedleHeight(100);
    QPolygon pointer;
    pointer << QPoint(0, -c.getNeedleHeight()) << QPoint(-c.getNeedleWidth(), 0) << QPoint(c.getNeedleWidth(), 0);

    // defines a path for the brush to take to draw the polygon
    QPainterPath path;
    path.addPolygon(pointer);


    //rotates the orientation of the painter so that the next objects will be rotated
    c.setNeedleAngle(-45);
    painter.rotate(c.getNeedleAngle());
    painter.drawPolygon(pointer);
    painter.fillPath(path, brush); //fills the polygon

    painter.rotate(-c.getNeedleAngle()); //resets orientation of painter

    //draw target
    c.setTargetAngle(80);
    pen.setBrush(Qt::red);
    painter.setBrush(Qt::red);
    painter.rotate(c.getTargetAngle());
    painter.drawEllipse(-10 , -c.getNeedleHeight() + -10 , 20 , 20);
    painter.rotate(-c.getTargetAngle());

}

void MainWindow::showTime(){

    QTime time = QTime::currentTime();

    QString time_text = time.toString("hh : mm : ss");
    if ((time.second() % 2) == 0){
        time_text[3] = ' ';
        time_text[8] = ' ';
        QWidget::update();
    }
    ui->Clock->setText(time_text);

}

MainWindow::~MainWindow()
{
    delete ui;
}

