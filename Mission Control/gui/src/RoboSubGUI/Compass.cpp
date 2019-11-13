#include "mainwindow.h"
#include "compass.h"
#include "ui_mainwindow.h"

#include <QPushButton>
#include <QLabel>
#include <QFrame>
#include <QTextBrowser>
#include <QTimer>
#include <QDateTime>

Compass::Compass(int centX, int centY) {
    centerX = centX;
    centerY = centY;

}

int Compass::getCenterX() {
    return centerX;
}

int Compass::getCenterY() {
    return centerY;
}

int Compass::getSquareWidth(){
    return width;
}

int Compass::getNeedleWidth(){
    return needleWidth;
}

int Compass::getNeedleHeight(){
    return needleHeight;
}

int Compass::getNeedleAngle(){
    return needleAngle;
}
int Compass::getTargetAngle(){
    return targetAngle;
}

void Compass::setSquareWidth(int sw){
    width = sw;
}

void Compass::setNeedleWidth(int nw){
    needleWidth = nw;
}

void Compass::setNeedleHeight(int nh){
    needleHeight = nh;
}

void Compass::setNeedleAngle(int na){
    needleAngle = na;
}

void Compass::setTargetAngle(int ta){
    targetAngle = ta;
}
