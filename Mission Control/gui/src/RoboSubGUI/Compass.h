#include <mainwindow.h>

#ifndef COMPASS_H
#define COMPASS_H


class Compass : MainWindow{
public:
    Compass(int centerX, int centerY);

    int getCenterX();
    int getCenterY();
    int getSquareWidth();
    int getNeedleWidth();
    int getNeedleHeight();
    int getNeedleAngle();
    int getTargetAngle();

    void setSquareWidth(int sw);
    void setNeedleWidth(int nw);
    void setNeedleHeight(int nh);
    void setNeedleAngle(int na);
    void setTargetAngle(int ta);




private:
    int centerX;
    int centerY;
    int width;
    int needleWidth;
    int needleHeight;
    int needleAngle;
    int targetAngle;

};





#endif // COMPASS_H
